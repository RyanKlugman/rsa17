/*
 * astar.hpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_ASTAR_HPP_
#define CROSBOT_ASTAR_HPP_

#include <crosbot/config.hpp>
#include <crosbot/data.hpp>
#include <rsa17/astar/astarParameters.hpp>
#include <crosbot_map/voronoi.hpp>

namespace crosbot {

typedef std::vector<Pose> Plan;
typedef std::vector<Index2D> CellPath;

class PathPlanner {
public:
	PathPlanner() {}
	virtual ~PathPlanner() {}
	virtual Plan getPath(VoronoiGridPtr grid, Pose start, Pose goal, ImagePtr);
	virtual CellPath getCellPath(VoronoiGridPtr grid, Index2D start, Index2D goal, ImagePtr)=0;
};

class AStarPlanner: public PathPlanner {
protected:

	double calculateCellRisk(const VoronoiGrid::VoronoiCell& cell, int maxExpandC, double expansionScale);

public:
	/*
	 * Cell risk options
	 * horizonDiscount: a reduction of risk for travelling along horizon
	 * k: risk multiplier, roughly equates to for every 1cm of travel touching wall
	 * 		it is preferred to travel kcm in an open area
	 * restrictedBonus: increased risk of travelling through restricted areas,
	 * 		roughly equates to for every 1cm of travel inside restricted its preffered
	 * 		to travel restrictedBonus cm touching wall or restrictedBonus*k cm in
	 * 		open area
	 */
	AStarParameters astarParameters;
	unsigned int cull;
	int maxExpandC;
	double expandScale;

	static Colour searchedSkeletonColour;
	static Colour searchedExpansionColour;
	static Colour searchedRestrictedColour;
	static Colour searchedPartialRestrictedColour;
	static Colour searchedVacantColour;
	static Colour pathColour;

	AStarPlanner();
	virtual ~AStarPlanner();

	void setParamaters(const AStarParameters& astarParameters);
	AStarParameters getParameters();

	virtual CellPath getCellPath(VoronoiGridPtr grid, Index2D start, Index2D goal, ImagePtr);

private:
	Index2D rayTraceNewGoal(VoronoiGridPtr grid, Index2D startCell, Index2D goalCell);
};

} // namespace crosbot

#endif /* ASTAR_HPP_ */
