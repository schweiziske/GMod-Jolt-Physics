#pragma once

#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <vector>

// Arc length is the path parameter, in meters. Jolt's position motor interprets
// its positional error as a distance; Hermite's raw node parameter does not have
// this property. The immutable table is built once, never in the physics step.
class VJoltPathSpline final : public JPH::PathConstraintPath
{
    JPH_DECLARE_RTTI_VIRTUAL(, VJoltPathSpline)
public:
    struct Node { JPH::Vec3 position, tangent, normal; };
    static constexpr uint32_t SamplesPerSegment = 64;
    std::vector<Node> nodes;

    bool Build(const std::vector<JPH::Vec3> &positions, float smoothing, bool closed)
    {
        SetIsLooping(closed);
        nodes.resize(positions.size());
        const size_t count = nodes.size();
        if (count < (closed ? 3u : 2u) || count > 32) return false;
        const float minChord = 0.0254f; // One Source unit.
        for (size_t i = 0; i < count; ++i)
        {
            const bool first = i == 0, last = i + 1 == count;
            JPH::Vec3 before = positions[i] - positions[(i + count - 1) % count];
            JPH::Vec3 after = positions[(i + 1) % count] - positions[i];
            if (!closed && first) before = after;
            if (!closed && last) after = before;
            const float a = before.Length(), b = after.Length();
            if (a < minChord || b < minChord) return false;
            const auto incoming = before / a, outgoing = after / b;
            // A reversal creates a cusp. Every accepted derivative has a strictly
            // positive projection onto its segment chord, so it cannot vanish.
            if (incoming.Dot(outgoing) < -0.98f) return false;
            nodes[i] = {positions[i], (incoming + outgoing).Normalized() * (std::min(a, b) * smoothing), {}};
        }
        JPH::Vec3 lastTangent = nodes[0].tangent.Normalized();
        JPH::Vec3 normal = JPH::Vec3::sAxisZ() - lastTangent * lastTangent.GetZ();
        if (normal.LengthSq() < .01f) normal = lastTangent.GetNormalizedPerpendicular();
        normal = normal.Normalized();
        for (auto &node : nodes)
        {
            const auto tangent = node.tangent.Normalized();
            normal = JPH::Quat::sFromTo(lastTangent, tangent) * normal;
            node.normal = (normal - tangent * normal.Dot(tangent)).Normalized();
            lastTangent = tangent;
        }
        if (closed)
        {
            // Distribute the parallel-transport seam twist over the loop.
            const auto firstTangent = nodes[0].tangent.Normalized();
            const auto endNormal = JPH::Quat::sFromTo(lastTangent, firstTangent) * nodes.back().normal;
            const float angle = std::atan2(firstTangent.Dot(endNormal.Cross(nodes[0].normal)),
                endNormal.Dot(nodes[0].normal));
            for (size_t i = 1; i < count; ++i)
                nodes[i].normal = JPH::Quat::sRotation(nodes[i].tangent.Normalized(), angle * float(i) / float(count)) * nodes[i].normal;
        }
        Rebuild();
        return !mDistances.empty() && mDistances.back() > minChord;
    }

    float GetPathMaxFraction() const override { return mDistances.back(); }
    float GetClosestPoint(JPH::Vec3Arg position, float hint) const override
    {
        const size_t first = std::min(size_t(ToFraction(hint)), SegmentCount() - 1);
        float bestSquared = FLT_MAX, bestDistance = 0, bestHintDistance = FLT_MAX;
        constexpr float tie = 1.0e-6f;
        for (size_t offset = 0; offset < SegmentCount(); ++offset)
        {
            const size_t i = (first + offset) % SegmentCount();
            const auto &segment = mSegments[i];
            const auto outside = JPH::Vec3::sMax(segment.minimum - position, JPH::Vec3::sZero())
                + JPH::Vec3::sMin(segment.maximum - position, JPH::Vec3::sZero());
            if (outside.LengthSq() > bestSquared + tie) continue;
            const float t = segment.hermite->GetClosestPoint(position, 0);
            JPH::Vec3 point, tangent; Evaluate(i, t, point, tangent);
            const float squared = (point - position).LengthSq();
            const float distance = ToDistance(float(i) + t);
            float hintDistance = std::abs(distance - hint);
            if (IsLooping()) hintDistance = std::min(hintDistance, GetPathMaxFraction() - hintDistance);
            // At a crossing, stay on the current branch when two candidates are
            // geometrically indistinguishable. A real teleport still chooses the
            // global nearest point. Cubic Bezier hulls bound the search exactly.
            if (squared < bestSquared - tie || (std::abs(squared - bestSquared) <= tie && hintDistance < bestHintDistance))
            {
                bestSquared = squared; bestDistance = distance; bestHintDistance = hintDistance;
            }
        }
        return bestDistance;
    }
    void GetPointOnPath(float distance, JPH::Vec3 &position, JPH::Vec3 &tangent,
        JPH::Vec3 &normal, JPH::Vec3 &binormal) const override
    {
        const float fraction = ToFraction(distance);
        const size_t i = std::min(size_t(fraction), SegmentCount() - 1);
        const float t = fraction - float(i);
        Evaluate(i, t, position, tangent);
        tangent = tangent.Normalized();
        normal = JPH::Quat::sFromTo(mSegments[i].tangent, tangent) * nodes[i].normal;
        normal = JPH::Quat::sRotation(tangent, mSegments[i].twist * t) * normal;
        binormal = normal.Cross(tangent).Normalized();
        normal = tangent.Cross(binormal);
    }
    void SaveBinaryState(JPH::StreamOut &stream) const override
    {
        PathConstraintPath::SaveBinaryState(stream);
        stream.Write(uint32_t(nodes.size()));
        for (const auto &node : nodes)
        {
            stream.Write(node.position); stream.Write(node.tangent); stream.Write(node.normal);
        }
    }
protected:
    void RestoreBinaryState(JPH::StreamIn &stream) override
    {
        PathConstraintPath::RestoreBinaryState(stream);
        uint32_t count = 0; stream.Read(count);
        if (count < 2 || count > 32 || stream.IsFailed()) return;
        nodes.resize(count);
        for (auto &node : nodes)
        {
            stream.Read(node.position); stream.Read(node.tangent); stream.Read(node.normal);
        }
        if (!stream.IsFailed()) Rebuild();
    }
private:
    size_t SegmentCount() const { return IsLooping() ? nodes.size() : nodes.size() - 1; }
    void Evaluate(size_t i, float t, JPH::Vec3 &position, JPH::Vec3 &tangent) const
    {
        const auto &a = nodes[i], &b = nodes[(i + 1) % nodes.size()];
        const float t2 = t * t, t3 = t2 * t;
        const auto chord = b.position - a.position;
        position = a.position + (t3 - 2*t2 + t)*a.tangent
            + (-2*t3 + 3*t2)*chord + (t3 - t2)*b.tangent;
        tangent = (3*t2 - 4*t + 1)*a.tangent
            + (-6*t2 + 6*t)*chord + (3*t2 - 2*t)*b.tangent;
    }
    void Rebuild()
    {
        mSegments.clear(); mSegments.reserve(SegmentCount());
        for (size_t i = 0; i < SegmentCount(); ++i)
        {
            const auto &a = nodes[i], &b = nodes[(i + 1) % nodes.size()];
            Segment segment;
            segment.hermite = new JPH::PathConstraintPathHermite;
            segment.hermite->AddPoint(a.position, a.tangent, a.normal);
            segment.hermite->AddPoint(b.position, b.tangent, b.normal);
            segment.minimum = JPH::Vec3::sMin(a.position, b.position);
            segment.maximum = JPH::Vec3::sMax(a.position, b.position);
            for (const auto &control : {a.position + a.tangent / 3, b.position - b.tangent / 3})
            {
                segment.minimum = JPH::Vec3::sMin(segment.minimum, control);
                segment.maximum = JPH::Vec3::sMax(segment.maximum, control);
            }
            segment.tangent = a.tangent.Normalized();
            const auto endTangent = b.tangent.Normalized();
            const auto endNormal = JPH::Quat::sFromTo(segment.tangent, endTangent) * a.normal;
            segment.twist = std::atan2(endTangent.Dot(endNormal.Cross(b.normal)), endNormal.Dot(b.normal));
            mSegments.push_back(std::move(segment));
        }
        mDistances.clear(); mDistances.reserve(SegmentCount() * SamplesPerSegment + 1);
        mDistances.push_back(0);
        JPH::Vec3 previous = nodes[0].position, position, tangent;
        for (size_t i = 0; i < SegmentCount(); ++i)
            for (uint32_t sample = 1; sample <= SamplesPerSegment; ++sample)
            {
                Evaluate(i, float(sample) / SamplesPerSegment, position, tangent);
                mDistances.push_back(mDistances.back() + (position - previous).Length());
                previous = position;
            }
    }
    float ToFraction(float distance) const
    {
        const float length = GetPathMaxFraction();
        if (IsLooping()) { distance = std::fmod(distance, length); if (distance < 0) distance += length; }
        else distance = std::clamp(distance, 0.0f, length);
        const auto upper = std::upper_bound(mDistances.begin(), mDistances.end(), distance);
        if (upper == mDistances.end()) return float(SegmentCount());
        const size_t index = size_t(upper - mDistances.begin() - 1);
        return (float(index) + (distance - mDistances[index]) / (*upper - mDistances[index])) / SamplesPerSegment;
    }
    float ToDistance(float fraction) const
    {
        const float sample = std::clamp(fraction * SamplesPerSegment, 0.0f, float(mDistances.size() - 1));
        const size_t index = std::min(size_t(sample), mDistances.size() - 2);
        return mDistances[index] + (sample - float(index)) * (mDistances[index + 1] - mDistances[index]);
    }
    struct Segment
    {
        JPH::Ref<JPH::PathConstraintPathHermite> hermite;
        JPH::Vec3 minimum, maximum, tangent;
        float twist;
    };
    std::vector<Segment> mSegments;
    std::vector<float> mDistances;
};
