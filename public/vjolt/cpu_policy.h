#pragma once
#include <cstdint>
#include <cstdlib>
#include <vector>
#ifdef _WIN32
#include <Windows.h>
#endif

namespace VJoltCPU
{
struct Core { uint64_t mask; unsigned efficiency; };
struct Policy
{
	uint64_t allowed = 0, owner = 0, reserved = 0, workers = 0;
	unsigned WorkerCount() const
	{
		unsigned count=0;
		for (auto bits=workers; bits; bits>>=1) count+=unsigned(bits&1);
		return count;
	}
};
// Pure selection is shared by the DLLs and exercised with synthetic SMT and
// hybrid topologies. Never isolate unless another physical core is available.
inline Policy Select(uint64_t allowed, const std::vector<Core> &cores, int requested=-1)
{
	Policy result{allowed,0,0,allowed};
	const Core *selected=nullptr;
	for (const auto &core:cores)
	{
		const auto usable=core.mask&allowed;
		if (!usable || !(allowed&~core.mask)) continue;
		if (requested>=0)
		{
			if (requested<64 && (usable&(uint64_t(1)<<requested))) { selected=&core; break; }
		}
		else if (!selected || core.efficiency>selected->efficiency) selected=&core;
	}
	if (!selected) return result;
	const auto usable=selected->mask&allowed;
	result.owner=requested>=0 ? uint64_t(1)<<requested : usable&(~usable+1);
	result.reserved=usable;
	result.workers=allowed&~selected->mask;
	return result;
}
inline Policy Discover()
{
#ifdef _WIN32
	DWORD_PTR allowed=0,system=0;
	if (!GetProcessAffinityMask(GetCurrentProcess(),&allowed,&system)) return {};
	Policy fallback{uint64_t(allowed),0,0,uint64_t(allowed)};
	// Group-aware assignment needs a separate policy; never misinterpret masks.
	if (GetActiveProcessorGroupCount()!=1) return fallback;
	const char *setting=std::getenv("VJOLT_SOURCE_CPU");
	int requested=-1;
	if (setting)
	{
		char *end=nullptr;
		const long parsed=std::strtol(setting,&end,10);
		if (*setting=='\0' || *end!='\0' || parsed<0 || parsed>63) return fallback;
		requested=int(parsed);
	}
	DWORD bytes=0;
	GetLogicalProcessorInformationEx(RelationProcessorCore,nullptr,&bytes);
	if (!bytes) return fallback;
	std::vector<unsigned char> data(bytes);
	auto *info=reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *>(data.data());
	if (!GetLogicalProcessorInformationEx(RelationProcessorCore,info,&bytes)) return fallback;
	std::vector<Core> cores;
	for (DWORD offset=0;offset<bytes;)
	{
		info=reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *>(data.data()+offset);
		if (!info->Size || info->Size>bytes-offset) return fallback;
		if (info->Processor.GroupCount==1 && info->Processor.GroupMask[0].Group==0)
			cores.push_back({uint64_t(info->Processor.GroupMask[0].Mask),info->Processor.EfficiencyClass});
		offset+=info->Size;
	}
	return Select(uint64_t(allowed),cores,requested);
#else
	return {};
#endif
}
inline const Policy &Get()
{
	// Read launch-time topology once. No mutex or topology enumeration per job.
	static const Policy policy=Discover();
	return policy;
}
inline void BindWorker()
{
#ifdef _WIN32
	const auto &policy=Get();
	if (policy.owner) SetThreadAffinityMask(GetCurrentThread(),DWORD_PTR(policy.workers));
#endif
}
class WorkerScope
{
public:
	WorkerScope()
	{
#ifdef _WIN32
		const auto &policy=Get();
		if (policy.owner) previous=SetThreadAffinityMask(GetCurrentThread(),DWORD_PTR(policy.workers));
#endif
	}
	~WorkerScope()
	{
#ifdef _WIN32
		if (previous) SetThreadAffinityMask(GetCurrentThread(),previous);
#endif
	}
private:
#ifdef _WIN32
	DWORD_PTR previous=0;
#endif
};
// Called from the actual engine Simulate entry, never guessed from CPU usage.
// Restore only if our assignment is still present; honor external affinity edits.
class OwnerBinding
{
public:
	void Bind()
	{
#ifdef _WIN32
		if (attempted) return;
		attempted=true;
		const auto &policy=Get();
		if (!policy.owner) return;
		if (!DuplicateHandle(GetCurrentProcess(),GetCurrentThread(),GetCurrentProcess(),&thread,0,FALSE,DUPLICATE_SAME_ACCESS)) return;
		assigned=DWORD_PTR(policy.owner);
		previous=SetThreadAffinityMask(thread,assigned);
#endif
	}
	void Reset()
	{
#ifdef _WIN32
		GROUP_AFFINITY affinity{};
		if (thread && previous && GetThreadGroupAffinity(thread,&affinity) && affinity.Group==0 && affinity.Mask==assigned)
			SetThreadAffinityMask(thread,previous);
		if(thread) CloseHandle(thread);
		thread=nullptr; previous=0; attempted=false;
#endif
	}
	~OwnerBinding() { Reset(); }
private:
#ifdef _WIN32
	HANDLE thread=nullptr;
	DWORD_PTR previous=0,assigned=0;
	bool attempted=false;
#endif
};
inline OwnerBinding &SourceOwner() { static OwnerBinding owner; return owner; }
} // namespace VJoltCPU
