#include "Common.h"
#include "PathFinding.h"
#include "MapTools.h"

namespace { auto & bwemMap = BWEM::Map::Instance(); }
namespace { auto & bwebMap = BWEB::Map::Instance(); }

using namespace UAlbertaBot;

int PathFinding::GetGroundDistance(BWAPI::Position start, BWAPI::Position end, PathFindingOptions options)
{
    // Parse options
    bool useNearestBWEMArea = ((int)options & (int)PathFindingOptions::UseNearestBWEMArea) != 0;

    // If either of the points is not in a BWEM area, fall back to air distance unless the caller overrides this
    if (!useNearestBWEMArea && (!bwemMap.GetArea(BWAPI::WalkPosition(start)) || !bwemMap.GetArea(BWAPI::WalkPosition(end))))
        return start.getApproxDistance(end);

    int dist;
    bwemMap.GetPath(start, end, &dist);
    return dist;
}

const BWEM::CPPath PathFinding::GetChokePointPath(BWAPI::Position start, BWAPI::Position end, PathFindingOptions options)
{
    // Parse options
    bool useNearestBWEMArea = ((int)options & (int)PathFindingOptions::UseNearestBWEMArea) != 0;

    // If either of the points is not in a BWEM area, it is probably over unwalkable terrain
    if (!useNearestBWEMArea && (!bwemMap.GetArea(BWAPI::WalkPosition(start)) || !bwemMap.GetArea(BWAPI::WalkPosition(end))))
        return BWEM::CPPath();

    return bwemMap.GetPath(start, end);
}

BWAPI::TilePosition PathFinding::NearbyPathfindingTile(BWAPI::TilePosition start)
{
    for (int radius = 0; radius < 4; radius++)
        for (int x = -radius; x <= radius; x++)
            for (int y = -radius; y <= radius; y++)
            {
                if (std::abs(x + y) != radius) continue;

                BWAPI::TilePosition tile = start + BWAPI::TilePosition(x, y);
                if (!tile.isValid()) continue;
                if (bwebMap.usedTilesGrid[tile.x][tile.y]) continue;
                if (!bwebMap.isWalkable(tile)) continue;
                return tile;
            }
    return BWAPI::TilePositions::Invalid;
}

std::vector<const BWEM::ChokePoint *> PathFinding::GetChokePointPathAvoidingUndesirableChokePoints(
    BWAPI::Position start,
    BWAPI::Position target,
    PathFindingOptions options,
    int minChokeWidth,
    int desiredChokeWidth)
{
    // Parse options
    bool useNearestBWEMArea = ((int)options & (int)PathFindingOptions::UseNearestBWEMArea) != 0;

    const BWEM::Area * startArea = useNearestBWEMArea ? bwemMap.GetNearestArea(BWAPI::WalkPosition(start)) : bwemMap.GetArea(BWAPI::WalkPosition(start));
    const BWEM::Area * targetArea = useNearestBWEMArea ? bwemMap.GetNearestArea(BWAPI::WalkPosition(target)) : bwemMap.GetArea(BWAPI::WalkPosition(target));
    if (!startArea || !targetArea) return {};

    struct Node {
        Node(const BWEM::ChokePoint * choke, int const dist, const BWEM::Area * toArea, const BWEM::ChokePoint * parent)
            : choke{ choke }, dist{ dist }, toArea{ toArea }, parent{ parent } { }
        mutable const BWEM::ChokePoint * choke;
        mutable int dist;
        mutable const BWEM::Area * toArea;
        mutable const BWEM::ChokePoint * parent = nullptr;
    };

    const auto validChoke = [](const BWEM::ChokePoint * choke, int minChokeWidth) {
        return !choke->Blocked() && 
            !((ChokeData*)choke->Ext())->requiresMineralWalk && 
            ((ChokeData*)choke->Ext())->width >= minChokeWidth;
    };

    const auto chokeDist = [](const BWEM::ChokePoint * choke, int dist, int desiredChokeWidth) {
        // Give too narrow chokes a large penalty, so they are only used if there is no other option
        if (((ChokeData*)choke->Ext())->width < desiredChokeWidth) return dist + 2000;
        return dist;
    };

    const auto chokeTo = [](const BWEM::ChokePoint * choke, const BWEM::Area * from) {
        return (from == choke->GetAreas().first)
            ? choke->GetAreas().second
            : choke->GetAreas().first;
    };

    const auto createPath = [](const Node& node, std::map<const BWEM::ChokePoint *, const BWEM::ChokePoint *> & parentMap) {
        std::vector<const BWEM::ChokePoint *> path;
        const BWEM::ChokePoint * current = node.choke;

        while (current)
        {
            path.push_back(current);
            current = parentMap[current];
        }

        std::reverse(path.begin(), path.end());

        return path;
    };

    auto cmp = [](Node left, Node right) { return left.dist > right.dist; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> nodeQueue(cmp);
    for (auto choke : startArea->ChokePoints())
        if (validChoke(choke, minChokeWidth))
            nodeQueue.emplace(
                choke,
                chokeDist(choke, start.getApproxDistance(BWAPI::Position(choke->Center())), desiredChokeWidth),
                chokeTo(choke, startArea),
                nullptr);

    std::map<const BWEM::ChokePoint *, const BWEM::ChokePoint *> parentMap;

    while (!nodeQueue.empty()) {
        auto const current = nodeQueue.top();
        nodeQueue.pop();

        // Set parent
        parentMap[current.choke] = current.parent;

        // If at target, return path
        // We're ignoring the distance from this last choke to the target position; it's an unlikely
        // edge case that there is an alternate choke giving a significantly better result
        if (current.toArea == targetArea)
            return createPath(current, parentMap);

        // Add valid connected chokes we haven't visited yet
        for (auto choke : current.toArea->ChokePoints())
            if (validChoke(choke, minChokeWidth) && parentMap.find(choke) == parentMap.end())
                nodeQueue.emplace(
                    choke,
                    chokeDist(choke, current.dist + choke->DistanceFrom(current.choke), desiredChokeWidth),
                    chokeTo(choke, current.toArea),
                    current.choke);
    }

    return {};
}

std::vector<const BWAPI::Position> UAlbertaBot::PathFinding::GetChokePointPathFarthest(BWAPI::Position start, BWAPI::Position target)
{
	std::vector<const BWAPI::Position> result_path;
	double mapWidth = bwemMap.Size().x;
	double mapHeight = bwemMap.Size().y;
	double hdist = std::abs(start.x - target.x);
	double vdist = std::abs(start.y - target.y);
	const std::set<BWTA::Chokepoint *>& cPoints = BWTA::getChokepoints();
	// choose the direction to go
	const BWEM::Area * start_area = bwemMap.GetArea(BWAPI::TilePosition(start));
	bwem_assert(start_area != nullptr);
	BWTA::Chokepoint * target_choke;
	int min_distance = 1 << 30;
	if (vdist < hdist)
	{
		double end_y = (target.y - 0 > mapHeight - target.y) ? 0. : mapHeight;
		BWAPI::Position vend_position(target.x, end_y);
		for (BWTA::Chokepoint * choke : cPoints)
		{
			int distance = GetGroundDistance(choke->getCenter(), vend_position);
			if (distance == -1) continue;
			if (distance < min_distance)
			{
				target_choke = choke;
				min_distance = distance;
			}
		}
		result_path.push_back(target_choke->getCenter());
	}
	else
	{
		double end_x = (target.x - 0 > mapWidth - target.x) ? 0. : mapWidth;
		BWAPI::Position hend_position(end_x, target.y);
		for (BWTA::Chokepoint * choke : cPoints)
		{
			double distance = GetGroundDistance(choke->getCenter(), hend_position);
			if (distance == -1) continue;
			if (distance < min_distance)
			{
				target_choke = choke;
				min_distance = distance;
			}
		}
		result_path.push_back(target_choke->getCenter());
	}
	result_path.push_back(target);
	return result_path;
}
