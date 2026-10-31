#pragma once

#include <cstddef>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "filesystem_helpers.h"
#if !defined( GAME_L4D2_OR_NEWER ) || defined( GAME_GMOD )
#include "tier1/KeyValues.h"
#else
#include "tier1/keyvalues.h"
#endif

namespace vjolt_keyvalues
{
struct ParseError
{
	const char *message = nullptr;
	std::size_t offset = 0;
};

struct DeleteKeyValues
{
	void operator()( KeyValues *value ) const
	{
		if ( value )
			value->deleteThis();
	}
};

using OwnedKeyValues = std::unique_ptr< KeyValues, DeleteKeyValues >;

namespace detail
{
enum class Token { End, Text, Open, Close, Invalid };

class Reader
{
public:
	explicit Reader( const char *text )
		: m_Start( text ? text : "" ), m_End( m_Start + std::strlen( m_Start ) ),
		  m_Cursor( m_Start ), m_Text( std::size_t( m_End - m_Start ) + 1 )
	{
	}

	Token Next( ParseError &error )
	{
		m_Offset = std::size_t( ( m_Cursor ? m_Cursor : m_End ) - m_Start );
		bool quoted = false;
		// This is the tokenizer used by original VPhysics ParseKeyvalue. In
		// particular, it skips both // and /* */ comments outside quoted tokens.
		m_Cursor = ParseFileInternal( m_Cursor, m_Text.data(), &quoted, nullptr, m_Text.size() );
		// ParseFile returns one past the terminating NUL for an unfinished quote.
		// Reject it before a subsequent call could read beyond the input.
		if ( m_Cursor && m_Cursor > m_End )
		{
			error = { "unterminated quoted string", m_Offset };
			return Token::Invalid;
		}
		if ( !m_Cursor )
			return Token::End;
		if ( !quoted && m_Text[ 1 ] == '\0' )
		{
			if ( m_Text[ 0 ] == '{' ) return Token::Open;
			if ( m_Text[ 0 ] == '}' ) return Token::Close;
		}
		return Token::Text;
	}

	const char *Text() const { return m_Text.data(); }
	void LowercaseText()
	{
		// ParseKeyvalue lowercases keys and values, using ASCII semantics.
		for ( char *p = m_Text.data(); *p; ++p )
			if ( *p >= 'A' && *p <= 'Z' ) *p += 'a' - 'A';
	}
	std::size_t Offset() const { return m_Offset; }

private:
	const char *m_Start;
	const char *m_End;
	const char *m_Cursor;
	std::vector< char > m_Text;
	std::size_t m_Offset = 0;
};
}

// Physics and surface descriptions expose named blocks, skipping scalar pairs
// outside them just like Source's NextBlock and ParseSurfaceData. Keep Source's
// tokenization without a second parse by KeyValues::LoadFromBuffer, which can
// report errors and return true.
// The caller owns the returned tree. A failure never returns a partial tree.
inline KeyValues *ParseHeaderless( const char *text, ParseError &error, bool lowercase = false )
{
	error = {};
	detail::Reader reader( text );
	OwnedKeyValues root( new KeyValues( "PhysProps" ) );
	struct Frame { KeyValues *parent; KeyValues *last; };
	std::vector< Frame > frames{ { root.get(), nullptr } };
	constexpr std::size_t maxDepth = 64;

	for ( ;; )
	{
		const auto key = reader.Next( error );
		if ( key == detail::Token::Invalid )
			return nullptr;
		if ( key == detail::Token::End )
		{
			if ( frames.size() != 1 )
			{
				error = { "missing closing brace", reader.Offset() };
				return nullptr;
			}
			return root.release();
		}
		if ( key == detail::Token::Close && frames.size() > 1 )
		{
			frames.pop_back();
			continue;
		}
		if ( key != detail::Token::Text || reader.Text()[ 0 ] == '\0' )
		{
			error = { "expected a key name", reader.Offset() };
			return nullptr;
		}
		if ( lowercase ) reader.LowercaseText();
		const std::string name( reader.Text() );
		const auto value = reader.Next( error );
		if ( value == detail::Token::Invalid )
			return nullptr;
		if ( value != detail::Token::Text && value != detail::Token::Open )
		{
			error = { "expected a value or opening brace", reader.Offset() };
			return nullptr;
		}
		if ( frames.size() == 1 && value != detail::Token::Open )
		{
			// Original VPhysics keeps looking for a block after a top-level
			// scalar pair. It must not discard solids/vehicles already read or
			// expose this metadata as an empty physics block.
			continue;
		}
		if ( value == detail::Token::Open && frames.size() > maxDepth )
		{
			error = { "block nesting limit exceeded", reader.Offset() };
			return nullptr;
		}

		OwnedKeyValues child( new KeyValues( name.c_str() ) );
		if ( value == detail::Token::Text )
		{
			if ( lowercase ) reader.LowercaseText();
			child->SetString( nullptr, reader.Text() );
		}
		auto &frame = frames.back();
		// Append in linear time and preserve repeated blocks and properties.
		if ( frame.last )
			frame.last->SetNextKey( child.get() );
		else
			frame.parent->AddSubKey( child.get() );
		frame.last = child.release();
		if ( value == detail::Token::Open )
			frames.push_back( { frame.last, nullptr } );
	}
}
}
