#ifndef _j1Pathfinfing_
#define _j1Pathfinding_

#include "PugiXml/src/pugixml.hpp"
#include "p2List.h"
#include "p2DynArray.h"
#include "p2PQueue.h"
#include "p2Point.h"
#include "j1Module.h"

#define COST_MAP 100




class j1Pathfinding : public j1Module
{
	public:
		j1Pathfinding();
		~j1Pathfinding();

		bool Awake(pugi::xml_node&);
		bool Start();
		// Pathfinding
		int MovementCost(int x, int y) const;
		void ResetPath();
		
		void Path(int x, int y);

		// Propagation style
		void PropagateBFS();
		void PropagateDijkstra();
		void PropagateAstar();
		void DrawPath();
	private:
		bool patfinding = false;
		/// BFS
		p2PQueue<iPoint>	frontier;
		p2List<iPoint>		visited;
		p2List<iPoint>		breadcrumbs;
		uint				cost_so_far[COST_MAP][COST_MAP];
		p2DynArray<iPoint>	path;
		SDL_Texture*		tile_x = nullptr;
};










#endif
