#include "../include/Voronoi.h"
#include "../include/PhysicsWorld.h"
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <random>

namespace IcePhysics
{
    VoronoiGenerator::VoronoiGenerator()
        : m_relaxationIterations(3)
        , m_epsilon(Math::EPSILON)
    {
    }

    VoronoiGenerator::~VoronoiGenerator()
    {
    }

    void VoronoiGenerator::Generate(const std::vector<Vector2>& points, const AABB& bounds, VoronoiDiagram& outDiagram)
    {
        std::vector<VoronoiSite> sites;
        sites.reserve(points.size());
        for (uint32_t i = 0; i < points.size(); i++)
        {
            VoronoiSite site;
            site.position = points[i];
            site.id = i;
            site.weight = 1.0f;
            sites.push_back(site);
        }
        GenerateWeighted(sites, bounds, outDiagram);
    }

    void VoronoiGenerator::GenerateWeighted(const std::vector<VoronoiSite>& sites, const AABB& bounds, VoronoiDiagram& outDiagram)
    {
        if (sites.empty())
        {
            return;
        }

        outDiagram.sites = sites;
        outDiagram.bounds = bounds;
        outDiagram.edges.clear();
        outDiagram.cells.clear();

        FortuneAlgorithm(sites, bounds, outDiagram);

        if (m_relaxationIterations > 0)
        {
            RelaxDiagram(outDiagram, m_relaxationIterations);
        }
    }

    void VoronoiGenerator::RelaxDiagram(VoronoiDiagram& diagram, uint32_t iterations)
    {
        if (diagram.sites.empty()) return;

        for (uint32_t iter = 0; iter < iterations; iter++)
        {
            BuildCells(diagram);

            std::vector<VoronoiSite> newSites;
            newSites.reserve(diagram.sites.size());

            for (const auto& cell : diagram.cells)
            {
                VoronoiSite site;
                site.id = cell.siteId;
                site.weight = diagram.sites[cell.siteId].weight;

                if (cell.area > m_epsilon)
                {
                    site.position = CalculateCellCentroid(cell.vertices, cell.area);
                }
                else
                {
                    site.position = cell.sitePosition;
                }

                Vector2 minBound(diagram.bounds.min.x, diagram.bounds.min.y);
                Vector2 maxBound(diagram.bounds.max.x, diagram.bounds.max.y);
                site.position.x = Math::Clamp(site.position.x, minBound.x + m_epsilon, maxBound.x - m_epsilon);
                site.position.y = Math::Clamp(site.position.y, minBound.y + m_epsilon, maxBound.y - m_epsilon);

                newSites.push_back(site);
            }

            diagram.sites = newSites;
            diagram.edges.clear();
            diagram.cells.clear();

            FortuneAlgorithm(diagram.sites, diagram.bounds, diagram);
        }

        BuildCells(diagram);
    }

    void VoronoiGenerator::FortuneAlgorithm(const std::vector<VoronoiSite>& sites, const AABB& bounds, VoronoiDiagram& outDiagram)
    {
        m_eventQueue.clear();
        m_arcs.clear();
        m_vertices.clear();
        m_edgeHalfEdges.clear();

        for (uint32_t i = 0; i < sites.size(); i++)
        {
            AddSiteEvent(i, sites[i].position);
        }

        SortEvents();

        while (!m_eventQueue.empty())
        {
            if (!ProcessNextEvent(sites, bounds, outDiagram))
            {
                break;
            }
        }

        ClipDiagramToBounds(outDiagram, bounds);
        BuildCells(outDiagram);
    }

    void VoronoiGenerator::AddSiteEvent(uint32_t siteIndex, const Vector2& position)
    {
        FortuneEvent evt;
        evt.type = FortuneEvent::Type::Site;
        evt.y = position.y;
        evt.siteIndex = siteIndex;
        evt.point = position;
        evt.arcIndex = 0xFFFFFFFF;
        m_eventQueue.push_back(evt);
    }

    void VoronoiGenerator::AddCircleEvent(uint32_t arcIndex, const Vector2& center, float y)
    {
        FortuneEvent evt;
        evt.type = FortuneEvent::Type::Circle;
        evt.y = y;
        evt.siteIndex = 0xFFFFFFFF;
        evt.point = center;
        evt.arcIndex = arcIndex;
        m_eventQueue.push_back(evt);
        m_arcs[arcIndex].circleEventIndex = (uint32_t)m_eventQueue.size() - 1;
    }

    bool VoronoiGenerator::ProcessNextEvent(const std::vector<VoronoiSite>& sites, const AABB& bounds, VoronoiDiagram& outDiagram)
    {
        if (m_eventQueue.empty()) return false;

        uint32_t topIndex = SortEvents();
        FortuneEvent evt = m_eventQueue[topIndex];

        std::swap(m_eventQueue[topIndex], m_eventQueue.back());
        m_eventQueue.pop_back();

        if (evt.type == FortuneEvent::Type::Site)
        {
            uint32_t newArc = CreateArc(evt.siteIndex);

            if (m_arcs.empty())
            {
                m_arcs.push_back(m_arcs[newArc]);
            }
            else
            {
                uint32_t aboveArc = FindArcAbove(evt.point.x, evt.point.y, sites);
                InsertArc(newArc, aboveArc);
            }

            if (m_arcs.size() > 1)
            {
                for (uint32_t i = 0; i < m_arcs.size(); i++)
                {
                    if (m_arcs[i].isActive)
                    {
                        CheckCircleEvent(i, sites);
                    }
                }
            }
        }
        else
        {
            RemoveArc(evt.arcIndex, sites, outDiagram);
        }

        return true;
    }

    uint32_t VoronoiGenerator::CreateArc(uint32_t siteIndex)
    {
        FortuneArc arc;
        arc.siteIndex = siteIndex;
        arc.prevArc = 0xFFFFFFFF;
        arc.nextArc = 0xFFFFFFFF;
        arc.s0 = -Math::PI;
        arc.s1 = Math::PI;
        arc.edgeIndex = 0xFFFFFFFF;
        arc.circleEventIndex = 0xFFFFFFFF;
        arc.isActive = true;
        m_arcs.push_back(arc);
        return (uint32_t)m_arcs.size() - 1;
    }

    void VoronoiGenerator::InsertArc(uint32_t newArcIndex, uint32_t existingArcIndex)
    {
        if (existingArcIndex >= m_arcs.size()) return;

        FortuneArc& newArc = m_arcs[newArcIndex];
        FortuneArc& existingArc = m_arcs[existingArcIndex];

        newArc.prevArc = existingArc.prevArc;
        newArc.nextArc = existingArcIndex;

        if (existingArc.prevArc != 0xFFFFFFFF)
        {
            m_arcs[existingArc.prevArc].nextArc = newArcIndex;
        }

        existingArc.prevArc = newArcIndex;

        uint32_t splitArc = CreateArc(existingArc.siteIndex);
        m_arcs[splitArc].prevArc = newArcIndex;
        m_arcs[splitArc].nextArc = existingArc.nextArc;
        m_arcs[splitArc].isActive = true;

        if (existingArc.nextArc != 0xFFFFFFFF)
        {
            m_arcs[existingArc.nextArc].prevArc = splitArc;
        }

        newArc.nextArc = splitArc;
    }

    void VoronoiGenerator::RemoveArc(uint32_t arcIndex, const std::vector<VoronoiSite>& sites, VoronoiDiagram& outDiagram)
    {
        if (arcIndex >= m_arcs.size() || !m_arcs[arcIndex].isActive) return;

        FortuneArc& arc = m_arcs[arcIndex];
        arc.isActive = false;

        if (arc.prevArc != 0xFFFFFFFF && arc.nextArc != 0xFFFFFFFF)
        {
            m_arcs[arc.prevArc].nextArc = arc.nextArc;
            m_arcs[arc.nextArc].prevArc = arc.prevArc;

            const Vector2& prevSite = sites[m_arcs[arc.prevArc].siteIndex].position;
            const Vector2& currSite = sites[arc.siteIndex].position;
            const Vector2& nextSite = sites[m_arcs[arc.nextArc].siteIndex].position;

            float ax = prevSite.x, ay = prevSite.y;
            float bx = currSite.x, by = currSite.y;
            float cx = nextSite.x, cy = nextSite.y;

            float d = 2.0f * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
            Vector2 vertex(currSite.x, currSite.y);
            if (Math::Abs(d) > m_epsilon)
            {
                float ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
                float uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
                vertex = Vector2(ux, uy);
            }
            m_vertices.push_back(vertex);

            if (arc.edgeIndex != 0xFFFFFFFF)
            {
                EndEdge(arc.edgeIndex, vertex, outDiagram);
            }

            if (m_arcs[arc.prevArc].edgeIndex != 0xFFFFFFFF)
            {
                EndEdge(m_arcs[arc.prevArc].edgeIndex, vertex, outDiagram);
            }

            CreateEdge(m_arcs[arc.prevArc].siteIndex, m_arcs[arc.nextArc].siteIndex, vertex, outDiagram);
        }
    }

    uint32_t VoronoiGenerator::FindArcAbove(float x, float y, const std::vector<VoronoiSite>& sites)
    {
        if (m_arcs.empty()) return 0xFFFFFFFF;

        uint32_t bestArc = 0;
        float bestY = -Math::PI;

        for (uint32_t i = 0; i < m_arcs.size(); i++)
        {
            if (!m_arcs[i].isActive) continue;

            const Vector2& sitePos = sites[m_arcs[i].siteIndex].position;
            float dx = x - sitePos.x;
            float dy = y - sitePos.y;

            if (Math::Abs(dy) < m_epsilon) continue;

            float parabolaY = sitePos.y + (dx * dx) / (2.0f * dy);
            if (parabolaY > bestY)
            {
                bestY = parabolaY;
                bestArc = i;
            }
        }

        return bestArc;
    }

    Vector2 VoronoiGenerator::GetIntersection(uint32_t arcIndex1, uint32_t arcIndex2, float y, const std::vector<VoronoiSite>& sites)
    {
        if (arcIndex1 >= m_arcs.size() || arcIndex2 >= m_arcs.size())
        {
            return Vector2(0, 0);
        }

        const Vector2& p1 = sites[m_arcs[arcIndex1].siteIndex].position;
        const Vector2& p2 = sites[m_arcs[arcIndex2].siteIndex].position;

        float a1 = 1.0f / (2.0f * (y - p1.y));
        float b1 = -p1.x * a1;
        float c1 = p1.y + (p1.x * p1.x) * a1;

        float a2 = 1.0f / (2.0f * (y - p2.y));
        float b2 = -p2.x * a2;
        float c2 = p2.y + (p2.x * p2.x) * a2;

        float a = a1 - a2;
        float b = b1 - b2;
        float c = c1 - c2;

        if (Math::Abs(a) < m_epsilon)
        {
            return Vector2(-c / b, y);
        }

        float discriminant = b * b - 4.0f * a * c;
        if (discriminant < 0.0f) discriminant = 0.0f;

        float x1 = (-b + Math::Sqrt(discriminant)) / (2.0f * a);
        float x2 = (-b - Math::Sqrt(discriminant)) / (2.0f * a);

        float x = (p1.x < p2.x) ? Math::Max(x1, x2) : Math::Min(x1, x2);

        return Vector2(x, y);
    }

    bool VoronoiGenerator::CheckCircleEvent(uint32_t arcIndex, const std::vector<VoronoiSite>& sites)
    {
        if (arcIndex >= m_arcs.size()) return false;

        FortuneArc& arc = m_arcs[arcIndex];

        if (arc.prevArc == 0xFFFFFFFF || arc.nextArc == 0xFFFFFFFF)
        {
            return false;
        }

        const Vector2& p = sites[arc.siteIndex].position;
        const Vector2& prevP = sites[m_arcs[arc.prevArc].siteIndex].position;
        const Vector2& nextP = sites[m_arcs[arc.nextArc].siteIndex].position;

        float ax = prevP.x, ay = prevP.y;
        float bx = p.x, by = p.y;
        float cx = nextP.x, cy = nextP.y;

        float d = 2.0f * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        if (Math::Abs(d) < m_epsilon) return false;

        float ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
        float uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax));
        uy /= d;

        float radius = Math::Sqrt((ax - ux) * (ax - ux) + (ay - uy) * (ay - uy));
        float eventY = uy + radius;

        if (eventY > p.y)
        {
            AddCircleEvent(arcIndex, Vector2(ux, uy), eventY);
            return true;
        }

        return false;
    }

    void VoronoiGenerator::CreateEdge(uint32_t siteA, uint32_t siteB, const Vector2& start, VoronoiDiagram& outDiagram)
    {
        VoronoiEdge edge;
        edge.start = start;
        edge.end = start;
        edge.siteA = siteA;
        edge.siteB = siteB;
        outDiagram.edges.push_back(edge);

        uint32_t edgeIndex = (uint32_t)outDiagram.edges.size() - 1;
        m_edgeHalfEdges.push_back(edgeIndex);
    }

    void VoronoiGenerator::EndEdge(uint32_t edgeIndex, const Vector2& end, VoronoiDiagram& outDiagram)
    {
        if (edgeIndex < outDiagram.edges.size())
        {
            outDiagram.edges[edgeIndex].end = end;
        }
    }

    void VoronoiGenerator::ClipDiagramToBounds(VoronoiDiagram& diagram, const AABB& bounds)
    {
        Vector2 minBound(bounds.min.x, bounds.min.y);
        Vector2 maxBound(bounds.max.x, bounds.max.y);

        for (auto& edge : diagram.edges)
        {
            ClipEdgeToBounds(edge.start, edge.end, bounds);
        }

        std::vector<VoronoiEdge> validEdges;
        for (const auto& edge : diagram.edges)
        {
            float dist = Math::Distance(edge.start, edge.end);
            if (dist > m_epsilon)
            {
                validEdges.push_back(edge);
            }
        }
        diagram.edges = validEdges;

        for (const auto& site : diagram.sites)
        {
            Vector2 pos = site.position;

            if (pos.x < minBound.x + m_epsilon || pos.x > maxBound.x - m_epsilon ||
                pos.y < minBound.y + m_epsilon || pos.y > maxBound.y - m_epsilon)
            {
                continue;
            }

            Vector2 corners[4] = {
                minBound,
                Vector2(maxBound.x, minBound.y),
                maxBound,
                Vector2(minBound.x, maxBound.y)
            };

            for (int i = 0; i < 4; i++)
            {
                Vector2 mid = (corners[i] + corners[(i + 1) % 4]) * 0.5f;
                Vector2 dir = corners[(i + 1) % 4] - corners[i];
                Vector2 normal(-dir.y, dir.x);
                normal = Math::Normalize(normal);

                Vector2 toSite = pos - mid;
                if (Math::Dot(toSite, normal) > 0)
                {
                    VoronoiEdge edge;
                    edge.start = corners[i];
                    edge.end = corners[(i + 1) % 4];
                    edge.siteA = site.id;
                    edge.siteB = 0xFFFFFFFF;
                    diagram.edges.push_back(edge);
                }
            }
        }
    }

    bool VoronoiGenerator::ClipEdgeToBounds(Vector2& start, Vector2& end, const AABB& bounds)
    {
        Vector2 minBound(bounds.min.x, bounds.min.y);
        Vector2 maxBound(bounds.max.x, bounds.max.y);

        float t0 = 0.0f;
        float t1 = 1.0f;
        Vector2 d = end - start;

        float p[4] = { -d.x, d.x, -d.y, d.y };
        float q[4] = { start.x - minBound.x, maxBound.x - start.x, start.y - minBound.y, maxBound.y - start.y };

        for (int i = 0; i < 4; i++)
        {
            if (Math::Abs(p[i]) < m_epsilon)
            {
                if (q[i] < 0.0f)
                {
                    return false;
                }
            }
            else
            {
                float t = q[i] / p[i];
                if (p[i] < 0.0f && t > t0)
                {
                    t0 = t;
                }
                else if (p[i] > 0.0f && t < t1)
                {
                    t1 = t;
                }
            }
        }

        if (t0 > t1) return false;

        Vector2 newStart = start + d * t0;
        Vector2 newEnd = start + d * t1;

        start = newStart;
        end = newEnd;

        return true;
    }

    bool VoronoiGenerator::LineIntersection(const Vector2& p1, const Vector2& p2, const Vector2& p3, const Vector2& p4, Vector2& intersection)
    {
        float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
        if (Math::Abs(den) < m_epsilon) return false;

        float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x));
        float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x));

        t /= den;
        u /= den;

        if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f)
        {
            intersection.x = p1.x + t * (p2.x - p1.x);
            intersection.y = p1.y + t * (p2.y - p1.y);
            return true;
        }

        return false;
    }

    void VoronoiGenerator::BuildCells(VoronoiDiagram& diagram)
    {
        diagram.cells.clear();
        diagram.cells.resize(diagram.sites.size());

        for (uint32_t i = 0; i < diagram.sites.size(); i++)
        {
            diagram.cells[i].siteId = i;
            diagram.cells[i].sitePosition = diagram.sites[i].position;
            diagram.cells[i].area = 0.0f;
        }

        for (uint32_t edgeIdx = 0; edgeIdx < diagram.edges.size(); edgeIdx++)
        {
            const auto& edge = diagram.edges[edgeIdx];

            if (edge.siteA < diagram.cells.size())
            {
                diagram.cells[edge.siteA].vertices.push_back(edge.start);
                diagram.cells[edge.siteA].vertices.push_back(edge.end);
                diagram.cells[edge.siteA].edges.push_back(edgeIdx);
            }

            if (edge.siteB < diagram.cells.size())
            {
                diagram.cells[edge.siteB].vertices.push_back(edge.end);
                diagram.cells[edge.siteB].vertices.push_back(edge.start);
                diagram.cells[edge.siteB].edges.push_back(edgeIdx);
            }
        }

        for (auto& cell : diagram.cells)
        {
            if (cell.vertices.size() < 3) continue;

            std::sort(cell.vertices.begin(), cell.vertices.end(),
                [&cell](const Vector2& a, const Vector2& b) {
                    float angleA = Math::Atan2(a.y - cell.sitePosition.y, a.x - cell.sitePosition.x);
                    float angleB = Math::Atan2(b.y - cell.sitePosition.y, b.x - cell.sitePosition.x);
                    return angleA < angleB;
                });

            std::vector<Vector2> uniqueVertices;
            for (const auto& v : cell.vertices)
            {
                if (uniqueVertices.empty() || Math::Distance(v, uniqueVertices.back()) > m_epsilon)
                {
                    uniqueVertices.push_back(v);
                }
            }
            cell.vertices = uniqueVertices;

            cell.area = CalculateCellArea(cell.vertices);

            cell.bounds.min = Vector3(cell.sitePosition.x, cell.sitePosition.y, 0);
            cell.bounds.max = Vector3(cell.sitePosition.x, cell.sitePosition.y, 0);

            for (const auto& v : cell.vertices)
            {
                cell.bounds.min.x = Math::Min(cell.bounds.min.x, v.x);
                cell.bounds.min.y = Math::Min(cell.bounds.min.y, v.y);
                cell.bounds.max.x = Math::Max(cell.bounds.max.x, v.x);
                cell.bounds.max.y = Math::Max(cell.bounds.max.y, v.y);
            }
        }
    }

    float VoronoiGenerator::CalculateCellArea(const std::vector<Vector2>& vertices)
    {
        if (vertices.size() < 3) return 0.0f;

        float area = 0.0f;
        uint32_t n = (uint32_t)vertices.size();

        for (uint32_t i = 0; i < n; i++)
        {
            uint32_t j = (i + 1) % n;
            area += vertices[i].x * vertices[j].y;
            area -= vertices[j].x * vertices[i].y;
        }

        return Math::Abs(area) * 0.5f;
    }

    Vector2 VoronoiGenerator::CalculateCellCentroid(const std::vector<Vector2>& vertices, float area)
    {
        if (vertices.size() < 3 || area < m_epsilon)
        {
            if (vertices.empty()) return Vector2(0, 0);
            return vertices[0];
        }

        float cx = 0.0f;
        float cy = 0.0f;
        uint32_t n = (uint32_t)vertices.size();

        for (uint32_t i = 0; i < n; i++)
        {
            uint32_t j = (i + 1) % n;
            float cross = vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
            cx += (vertices[i].x + vertices[j].x) * cross;
            cy += (vertices[i].y + vertices[j].y) * cross;
        }

        cx /= (6.0f * area);
        cy /= (6.0f * area);

        return Vector2(cx, cy);
    }

    uint32_t VoronoiGenerator::SortEvents()
    {
        if (m_eventQueue.size() <= 1)
        {
            return 0;
        }

        for (int i = (int)m_eventQueue.size() / 2 - 1; i >= 0; i--)
        {
            HeapifyDown((uint32_t)i);
        }

        return 0;
    }

    void VoronoiGenerator::HeapifyUp(uint32_t index)
    {
        while (index > 0)
        {
            uint32_t parent = (index - 1) / 2;
            if (m_eventQueue[index].y > m_eventQueue[parent].y)
            {
                std::swap(m_eventQueue[index], m_eventQueue[parent]);
                index = parent;
            }
            else
            {
                break;
            }
        }
    }

    void VoronoiGenerator::HeapifyDown(uint32_t index)
    {
        uint32_t size = (uint32_t)m_eventQueue.size();

        while (true)
        {
            uint32_t largest = index;
            uint32_t left = 2 * index + 1;
            uint32_t right = 2 * index + 2;

            if (left < size && m_eventQueue[left].y > m_eventQueue[largest].y)
            {
                largest = left;
            }

            if (right < size && m_eventQueue[right].y > m_eventQueue[largest].y)
            {
                largest = right;
            }

            if (largest != index)
            {
                std::swap(m_eventQueue[index], m_eventQueue[largest]);
                index = largest;
            }
            else
            {
                break;
            }
        }
    }

    IceSheet::IceSheet(const IceSheetDesc& desc)
        : m_id(0)
        , m_position(desc.position)
        , m_size(desc.size)
        , m_thickness(desc.thickness)
        , m_yieldStrength(desc.yieldStrength)
        , m_youngsModulus(desc.youngsModulus)
        , m_poissonsRatio(desc.poissonsRatio)
        , m_density(desc.density)
        , m_subdivisions(desc.subdivisions)
        , m_rigidBodyId(0)
        , m_colliderId(0)
        , m_nextFragmentId(1)
    {
        GenerateGridPoints();
    }

    IceSheet::~IceSheet()
    {
    }

    void IceSheet::GenerateMesh()
    {
        m_vertices.clear();
        m_triangles.clear();

        float halfX = m_size.x * 0.5f;
        float halfY = m_size.y * 0.5f;

        uint32_t segments = m_subdivisions + 1;

        for (uint32_t j = 0; j <= segments; j++)
        {
            for (uint32_t i = 0; i <= segments; i++)
            {
                float x = (float)i / (float)segments * m_size.x - halfX;
                float y = (float)j / (float)segments * m_size.y - halfY;

                Vertex v;
                v.position = Vector3(m_position.x + x, m_position.y, m_position.z + y);
                v.normal = Math::Vec3Up();
                v.uv = Vector2((float)i / (float)segments, (float)j / (float)segments);
                m_vertices.push_back(v);
            }
        }

        for (uint32_t j = 0; j < segments; j++)
        {
            for (uint32_t i = 0; i < segments; i++)
        {
                uint32_t idx0 = j * (segments + 1) + i;
                uint32_t idx1 = j * (segments + 1) + i + 1;
                uint32_t idx2 = (j + 1) * (segments + 1) + i;
                uint32_t idx3 = (j + 1) * (segments + 1) + i + 1;

                Triangle tri1, tri2;
                tri1.indices[0] = idx0;
                tri1.indices[1] = idx2;
                tri1.indices[2] = idx1;

                tri2.indices[0] = idx1;
                tri2.indices[1] = idx2;
                tri2.indices[2] = idx3;

                m_triangles.push_back(tri1);
                m_triangles.push_back(tri2);
            }
        }
    }

    bool IceSheet::GetMesh(uint32_t& vertexCount, Vertex* vertices, uint32_t& triangleCount, Triangle* triangles) const
    {
        vertexCount = (uint32_t)m_vertices.size();
        triangleCount = (uint32_t)m_triangles.size();

        if (vertices && vertexCount > 0)
        {
            memcpy(vertices, m_vertices.data(), vertexCount * sizeof(Vertex));
        }

        if (triangles && triangleCount > 0)
        {
            memcpy(triangles, m_triangles.data(), triangleCount * sizeof(Triangle));
        }

        return true;
    }

    bool IceSheet::CheckImpact(const IceImpactData& impact, float yieldStrength)
    {
        float bendingStress = CalculateBendingStress(impact.impactForce, 0.0f, m_thickness);
        float shearStress = CalculateShearStress(impact.impactForce, impact.contactArea);

        float maxStress = Math::Max(bendingStress, shearStress);

        return CheckStressFailure(maxStress, yieldStrength);
    }

    bool IceSheet::BreakAtPoint(const IceImpactData& impact, float yieldStrength,
                               uint32_t& fragmentCount, IceFragmentDesc*& fragments,
                               std::vector<IceFragment>& outFragments)
    {
        if (!CheckImpact(impact, yieldStrength))
        {
            fragmentCount = 0;
            fragments = nullptr;
            return false;
        }

        Vector2 impactPoint2D(impact.impactPoint.x - m_position.x, impact.impactPoint.z - m_position.z);

        float impactRadius = Math::Sqrt(impact.impactForce / (Math::PI * m_yieldStrength));
        impactRadius = Math::Clamp(impactRadius, 0.5f, Math::Min(m_size.x, m_size.y) * 0.5f);

        uint32_t siteCount = (uint32_t)(impact.impactForce / m_yieldStrength) + 5;
        siteCount = Math::Clamp(siteCount, 5u, 30u);

        std::vector<Vector2> sites = GenerateVoronoiSites(impactPoint2D, impactRadius, siteCount);

        AABB bounds;
        bounds.min = Vector3(m_position.x - m_size.x * 0.5f, m_position.y, m_position.z - m_size.y * 0.5f);
        bounds.max = Vector3(m_position.x + m_size.x * 0.5f, m_position.y, m_position.z + m_size.y * 0.5f);

        VoronoiDiagram diagram;
        m_voronoiGenerator.Generate(sites, bounds, diagram);

        Vector2 origin(m_position.x, m_position.z);

        uint32_t fragmentCollisionGroup = (uint32_t)(0x1 << ((m_id % 30) + 1));

        for (const auto& cell : diagram.cells)
        {
            if (cell.vertices.size() < 3) continue;

            IceFragment fragment;
            fragment.id = m_nextFragmentId++;
            fragment.isActive = true;
            fragment.lifetime = 30.0f;
            fragment.sourceSheetId = m_id;
            fragment.collisionGroup = fragmentCollisionGroup;
            fragment.collisionCooldown = 2.0f;
            fragment.bodyId = 0;
            fragment.colliderId = 0;
            fragment.isStatic = false;

            GenerateFragmentMesh(cell, origin, fragment);
            ExtrudeFragment(fragment, m_thickness);
            CalculateFragmentMassProperties(fragment);
            CalculateFragmentInertia(fragment);
            InitializeFragmentPhysics(fragment, impact);

            outFragments.push_back(fragment);
        }

        m_fragments = outFragments;

        fragmentCount = (uint32_t)outFragments.size();

        if (fragmentCount > 0)
        {
            fragments = new IceFragmentDesc[fragmentCount];
            for (uint32_t i = 0; i < fragmentCount; i++)
        {
                fragments[i].position = outFragments[i].position;
                fragments[i].rotation = outFragments[i].rotation;
                fragments[i].velocity = outFragments[i].velocity;
                fragments[i].angularVelocity = outFragments[i].angularVelocity;
                fragments[i].centerOfMass = outFragments[i].centerOfMass;
                fragments[i].mass = outFragments[i].mass;
                fragments[i].inertiaTensor = outFragments[i].inertiaTensor;
                fragments[i].vertexCount = (uint32_t)outFragments[i].vertices.size();
                fragments[i].triangleCount = (uint32_t)outFragments[i].triangles.size();
            }
        }

        return fragmentCount > 0;
    }

    void IceSheet::Update(float dt, float waterDensity, const Vector3& gravity)
    {
        for (auto& fragment : m_fragments)
        {
            if (!fragment.isActive) continue;

            ApplyBuoyancy(fragment, waterDensity, gravity);

            fragment.velocity += gravity * dt;

            float drag = 0.98f;
            fragment.velocity *= drag;
            fragment.angularVelocity *= drag;

            fragment.position += fragment.velocity * dt;

            float angularSpeed = Math::Length(fragment.angularVelocity);
            if (angularSpeed > Math::EPSILON)
            {
                Vector3 axis = Math::Normalize(fragment.angularVelocity);
                float angle = angularSpeed * dt;
                Quaternion deltaRot = Math::QuatFromAngleAxis(angle, axis);
                fragment.rotation = Math::QuatNormalize(fragment.rotation * deltaRot);
            }

            fragment.lifetime -= dt;
            if (fragment.position.y < -5.0f || fragment.lifetime <= 0.0f)
            {
                fragment.isActive = false;
            }

            fragment.bounds = Math::AABBFromCenterExtents(fragment.position, Vector3(1, 1, 1));
            for (const auto& v : fragment.vertices)
            {
                Vector3 worldPos = fragment.position + fragment.rotation * v.position;
                fragment.bounds = Math::AABBExpand(fragment.bounds, worldPos);
            }
        }
    }

    bool IceSheet::GetFragmentData(uint32_t fragmentIndex, IceFragmentDesc& desc,
                                 uint32_t& vertexCount, Vertex* vertices,
                                 uint32_t& triangleCount, Triangle* triangles) const
    {
        if (fragmentIndex >= m_fragments.size()) return false;

        const IceFragment& fragment = m_fragments[fragmentIndex];

        desc.position = fragment.position;
        desc.rotation = fragment.rotation;
        desc.velocity = fragment.velocity;
        desc.angularVelocity = fragment.angularVelocity;
        desc.centerOfMass = fragment.centerOfMass;
        desc.mass = fragment.mass;
        desc.inertiaTensor = fragment.inertiaTensor;

        vertexCount = (uint32_t)fragment.vertices.size();
        triangleCount = (uint32_t)fragment.triangles.size();
        desc.vertexCount = vertexCount;
        desc.triangleCount = triangleCount;

        if (vertices && vertexCount > 0)
        {
            memcpy(vertices, fragment.vertices.data(), vertexCount * sizeof(Vertex));
        }

        if (triangles && triangleCount > 0)
        {
            memcpy(triangles, fragment.triangles.data(), triangleCount * sizeof(Triangle));
        }

        return true;
    }

    const IceFragment* IceSheet::GetFragment(uint32_t index) const
    {
        if (index >= m_fragments.size()) return nullptr;
        return &m_fragments[index];
    }

    IceFragment* IceSheet::GetFragment(uint32_t index)
    {
        if (index >= m_fragments.size()) return nullptr;
        return &m_fragments[index];
    }

    void IceSheet::CalculatePressureDistribution(const Vector3& shipPosition, const Quaternion& shipRotation,
                                               uint32_t& pointCount, PressurePoint* pressurePoints)
    {
        pointCount = 0;

        if (pressurePoints == nullptr)
        {
            pointCount = (uint32_t)m_gridPoints.size();
            return;
        }

        float halfX = m_size.x * 0.5f;

        Vector3 localShipPos = shipPosition - m_position;
        localShipPos = Math::QuatInverse(shipRotation) * localShipPos;

        uint32_t count = (uint32_t)m_gridPoints.size();

        for (uint32_t i = 0; i < count; i++)
        {
            const Vector2& gp = m_gridPoints[i];
            Vector3 pointPos(gp.x, 0, gp.y);

            float dist = Math::Distance(pointPos, Vector3(localShipPos.x, 0, localShipPos.z));
            float pressure = 0.0f;
            float area = (m_size.x / (float)(m_subdivisions + 1)) * (m_size.y / (float)(m_subdivisions + 1));

            float influenceRadius = halfX * 0.5f;
            if (dist < influenceRadius)
            {
                float factor = 1.0f - (dist / influenceRadius);
                pressure = m_yieldStrength * factor * factor;
            }

            pressurePoints[i].position = m_position + Vector3(gp.x, 0, gp.y);
            pressurePoints[i].normal = Math::Vec3Up();
            pressurePoints[i].pressure = pressure;
            pressurePoints[i].area = area;

            if (pressure > Math::EPSILON)
            {
                pointCount++;
            }
        }
    }

    void IceSheet::GenerateGridPoints()
    {
        m_gridPoints.clear();

        float halfX = m_size.x * 0.5f;
        float halfY = m_size.y * 0.5f;

        for (uint32_t j = 0; j <= m_subdivisions; j++)
        {
            for (uint32_t i = 0; i <= m_subdivisions; i++)
        {
                float x = (float)i / (float)m_subdivisions * m_size.x - halfX;
                float y = (float)j / (float)m_subdivisions * m_size.y - halfY;
                m_gridPoints.push_back(Vector2(x, y));
            }
        }
    }

    std::vector<Vector2> IceSheet::GenerateVoronoiSites(const Vector2& impactPoint, float impactRadius, uint32_t siteCount)
    {
        std::vector<Vector2> sites;
        sites.reserve(siteCount);

        std::mt19937 rng(12345);
        std::uniform_real_distribution<float> distAngle(0.0f, Math::TWO_PI);
        std::uniform_real_distribution<float> distRadius(0.0f, 1.0f);

        sites.push_back(impactPoint);

        for (uint32_t i = 1; i < siteCount; i++)
        {
            float angle = distAngle(rng);
            float radius = Math::Sqrt(distRadius(rng)) * impactRadius;

            Vector2 offset(Math::Cos(angle) * radius, Math::Sin(angle) * radius);
            Vector2 site = impactPoint + offset;

            float halfX = m_size.x * 0.5f;
            float halfY = m_size.y * 0.5f;
            site.x = Math::Clamp(site.x, -halfX + 0.1f, halfX - 0.1f);
            site.y = Math::Clamp(site.y, -halfY + 0.1f, halfY - 0.1f);

            sites.push_back(site);
        }

        return sites;
    }

    void IceSheet::GenerateFragmentMesh(const VoronoiCell& cell, const Vector2& origin, IceFragment& fragment)
    {
        fragment.vertices.clear();
        fragment.triangles.clear();

        for (const auto& v : cell.vertices)
        {
            Vertex vertex;
            vertex.position = Vector3(v.x - origin.x, 0, v.y - origin.y);
            vertex.normal = Math::Vec3Up();
            vertex.uv = Vector2((v.x + m_size.x * 0.5f) / m_size.x, (v.y + m_size.y * 0.5f) / m_size.y);
            fragment.vertices.push_back(vertex);
        }

        uint32_t n = (uint32_t)cell.vertices.size();
        for (uint32_t i = 1; i < n - 1; i++)
        {
            Triangle tri;
            tri.indices[0] = 0;
            tri.indices[1] = i;
            tri.indices[2] = i + 1;
            fragment.triangles.push_back(tri);
        }
    }

    void IceSheet::ExtrudeFragment(IceFragment& fragment, float height)
    {
        if (fragment.vertices.empty()) return;

        uint32_t topCount = (uint32_t)fragment.vertices.size();

        for (uint32_t i = 0; i < topCount; i++)
        {
            Vertex v = fragment.vertices[i];
            v.position.y = -height;
            v.normal = Vector3(0, -1, 0);
            fragment.vertices.push_back(v);
        }

        for (uint32_t i = 0; i < topCount; i++)
        {
            uint32_t next = (i + 1) % topCount;

            Triangle tri1, tri2;
            tri1.indices[0] = i;
            tri1.indices[1] = i + topCount;
            tri1.indices[2] = next;

            tri2.indices[0] = next;
            tri2.indices[1] = i + topCount;
            tri2.indices[2] = next + topCount;

            fragment.triangles.push_back(tri1);
            fragment.triangles.push_back(tri2);
        }

        for (uint32_t i = 1; i < topCount - 1; i++)
        {
            Triangle tri;
            tri.indices[0] = topCount;
            tri.indices[1] = topCount + i + 1;
            tri.indices[2] = topCount + i;
            fragment.triangles.push_back(tri);
        }

        for (uint32_t i = 0; i < fragment.vertices.size(); i++)
        {
            fragment.vertices[i].position.y += height * 0.5f;
        }
    }

    void IceSheet::CalculateFragmentMassProperties(IceFragment& fragment)
    {
        if (fragment.vertices.empty()) return;

        float volume = 0.0f;
        Vector3 centerOfMass = Math::Vec3Zero();

        for (const auto& tri : fragment.triangles)
        {
            const Vector3& v0 = fragment.vertices[tri.indices[0]].position;
            const Vector3& v1 = fragment.vertices[tri.indices[1]].position;
            const Vector3& v2 = fragment.vertices[tri.indices[2]].position;

            float signedVol = Math::SignedVolume(v0, v1, v2, Math::Vec3Zero());
            volume += signedVol;

            Vector3 centroid = (v0 + v1 + v2) * 0.25f;
            centerOfMass += centroid * signedVol;
        }

        volume = Math::Abs(volume);
        fragment.mass = volume * m_density;

        if (volume > Math::EPSILON)
        {
            fragment.centerOfMass = centerOfMass / volume;
        }
        else
        {
            fragment.centerOfMass = Math::Vec3Zero();
        }

        fragment.position = m_position + fragment.centerOfMass;
        fragment.rotation = Math::QuatIdentity();
    }

    void IceSheet::CalculateFragmentInertia(IceFragment& fragment)
    {
        if (fragment.vertices.empty())
        {
            fragment.inertiaTensor = Math::Mat3Identity();
            return;
        }

        float Ixx = 0.0f, Iyy = 0.0f, Izz = 0.0f;
        float Ixy = 0.0f, Ixz = 0.0f, Iyz = 0.0f;

        for (const auto& tri : fragment.triangles)
        {
            const Vector3& v0 = fragment.vertices[tri.indices[0]].position;
            const Vector3& v1 = fragment.vertices[tri.indices[1]].position;
            const Vector3& v2 = fragment.vertices[tri.indices[2]].position;

            float signedVol = Math::SignedVolume(v0, v1, v2, Math::Vec3Zero());

            for (int i = 0; i < 3; i++)
            {
                float x0 = v0.x, y0 = v0.y, z0 = v0.z;
                float x1 = v1.x, y1 = v1.y, z1 = v1.z;
                float x2 = v2.x, y2 = v2.y, z2 = v2.z;

                Ixx += (y0 * y0 + y0 * y1 + y1 * y1 + y0 * y2 + y1 * y2 + y2 * y2 +
                        z0 * z0 + z0 * z1 + z1 * z1 + z0 * z2 + z1 * z2 + z2 * z2) * signedVol / 60.0f;
                Iyy += (x0 * x0 + x0 * x1 + x1 * x1 + x0 * x2 + x1 * x2 + x2 * x2 +
                        z0 * z0 + z0 * z1 + z1 * z1 + z0 * z2 + z1 * z2 + z2 * z2) * signedVol / 60.0f;
                Izz += (x0 * x0 + x0 * x1 + x1 * x1 + x0 * x2 + x1 * x2 + x2 * x2 +
                        y0 * y0 + y0 * y1 + y1 * y1 + y0 * y2 + y1 * y2 + y2 * y2) * signedVol / 60.0f;
                Ixy += (2 * x0 * y0 + x1 * y0 + x2 * y0 + x0 * y1 + 2 * x1 * y1 + x2 * y1 + x0 * y2 + x1 * y2 + 2 * x2 * y2) * signedVol / 120.0f;
                Ixz += (2 * x0 * z0 + x1 * z0 + x2 * z0 + x0 * z1 + 2 * x1 * z1 + x2 * z1 + x0 * z2 + x1 * z2 + 2 * x2 * z2) * signedVol / 120.0f;
                Iyz += (2 * y0 * z0 + y1 * z0 + y2 * z0 + y0 * z1 + 2 * y1 * z1 + y2 * z1 + y0 * z2 + y1 * z2 + 2 * y2 * z2) * signedVol / 120.0f;
            }
        }

        fragment.inertiaTensor.m[0] = Ixx * m_density;
        fragment.inertiaTensor.m[1] = -Ixy * m_density;
        fragment.inertiaTensor.m[2] = -Ixz * m_density;
        fragment.inertiaTensor.m[3] = -Ixy * m_density;
        fragment.inertiaTensor.m[4] = Iyy * m_density;
        fragment.inertiaTensor.m[5] = -Iyz * m_density;
        fragment.inertiaTensor.m[6] = -Ixz * m_density;
        fragment.inertiaTensor.m[7] = -Iyz * m_density;
        fragment.inertiaTensor.m[8] = Izz * m_density;
    }

    bool IceSheet::CheckStressFailure(float stress, float yieldStrength)
    {
        return stress > yieldStrength;
    }

    float IceSheet::CalculateBendingStress(float impactForce, float distance, float thickness)
    {
        if (thickness < Math::EPSILON) return 0.0f;

        float moment = impactForce * distance;
        float sectionModulus = (thickness * thickness * thickness) / 6.0f;
        return moment / sectionModulus;
    }

    float IceSheet::CalculateShearStress(float impactForce, float contactArea)
    {
        if (contactArea < Math::EPSILON) return 0.0f;
        return impactForce / contactArea;
    }

    void IceSheet::InitializeFragmentPhysics(IceFragment& fragment, const IceImpactData& impact)
    {
        Vector3 toFragment = fragment.position - impact.impactPoint;
        float dist = Math::Length(toFragment);
        Vector3 dir = Math::Normalize(toFragment);

        float impulseMagnitude = impact.impactForce * 0.1f / (1.0f + dist * 0.5f);
        fragment.velocity = dir * impulseMagnitude / Math::Max(fragment.mass, 0.1f);

        Vector3 angularDir = Math::Cross(impact.impactNormal, dir);
        fragment.angularVelocity = angularDir * Math::Sqrt(impulseMagnitude) * 0.5f;
    }

    void IceSheet::ApplyBuoyancy(IceFragment& fragment, float waterDensity, const Vector3& gravity)
    {
        float displacedVolume = 0.0f;
        Vector3 buoyancyCenter = Math::Vec3Zero();

        for (const auto& v : fragment.vertices)
        {
            Vector3 worldPos = fragment.position + fragment.rotation * v.position;
            if (worldPos.y < 0.0f)
            {
                float submerged = Math::Clamp(-worldPos.y, 0.0f, m_thickness);
                displacedVolume += submerged;
                buoyancyCenter += worldPos;
            }
        }

        if (displacedVolume > Math::EPSILON)
        {
            buoyancyCenter /= (float)fragment.vertices.size();

            float buoyantForce = waterDensity * displacedVolume * Math::Length(gravity);
            Vector3 force = Math::Normalize(gravity) * (-buoyantForce);

            fragment.velocity += force / fragment.mass * 0.016f;

            Vector3 torqueArm = buoyancyCenter - fragment.position;
            Vector3 torque = Math::Cross(torqueArm, force);
            fragment.angularVelocity += torque * 0.016f;
        }
    }

    IceSheetManager::IceSheetManager()
        : m_nextIceSheetId(1)
        , m_iceBreakCallback(nullptr)
    {
    }

    IceSheetManager::~IceSheetManager()
    {
        for (auto& pair : m_iceSheets)
        {
            delete pair.second;
        }
        m_iceSheets.clear();
    }

    uint32_t IceSheetManager::CreateIceSheet(const IceSheetDesc& desc)
    {
        IceSheet* sheet = new IceSheet(desc);
        sheet->SetId(m_nextIceSheetId);
        sheet->GenerateMesh();
        m_iceSheets[m_nextIceSheetId] = sheet;
        return m_nextIceSheetId++;
    }

    void IceSheetManager::DestroyIceSheet(uint32_t iceSheetId)
    {
        auto it = m_iceSheets.find(iceSheetId);
        if (it != m_iceSheets.end())
        {
            delete it->second;
            m_iceSheets.erase(it);
        }
    }

    IceSheet* IceSheetManager::GetIceSheet(uint32_t iceSheetId)
    {
        auto it = m_iceSheets.find(iceSheetId);
        if (it != m_iceSheets.end())
        {
            return it->second;
        }
        return nullptr;
    }

    const IceSheet* IceSheetManager::GetIceSheet(uint32_t iceSheetId) const
    {
        auto it = m_iceSheets.find(iceSheetId);
        if (it != m_iceSheets.end())
        {
            return it->second;
        }
        return nullptr;
    }

    void IceSheetManager::Update(float dt, float waterDensity, const Vector3& gravity)
    {
        for (auto& pair : m_iceSheets)
        {
            pair.second->Update(dt, waterDensity, gravity);
        }
    }

    bool IceSheetManager::HandleIceImpact(uint32_t iceSheetId, const IceImpactData& impact)
    {
        IceSheet* sheet = GetIceSheet(iceSheetId);
        if (!sheet) return false;

        uint32_t fragmentCount = 0;
        IceFragmentDesc* fragments = nullptr;
        std::vector<IceFragment> outFragments;

        bool broke = sheet->BreakAtPoint(impact, sheet->GetYieldStrength(), fragmentCount, fragments, outFragments);

        if (broke && m_iceBreakCallback && fragments)
        {
            m_iceBreakCallback(iceSheetId, impact.impactPoint, fragmentCount, fragments);
            delete[] fragments;
        }

        return broke;
    }
}
