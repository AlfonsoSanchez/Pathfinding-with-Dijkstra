#include <math.h>
#include "p2Defs.h"
#include "p2Log.h"
#include "j1App.h"
#include "j1Render.h"
#include "j1Textures.h"
#include "j1Pathfinding.h"
#include "j1Map.h"


j1Pathfinding::j1Pathfinding() :j1Module()
{
	name.create("pathfinding");

}

j1Pathfinding::~j1Pathfinding()
{ }

bool j1Pathfinding::Awake(pugi::xml_node& config)
{
	ResetPath();
	return true;
}

bool j1Pathfinding::Start()
{
	tile_x = App->tex->Load("maps/x.png");
	return true;
}

void j1Pathfinding::ResetPath()
{
	frontier.Clear();
	visited.clear();
	breadcrumbs.clear();
	frontier.Push(iPoint(19, 4), 0);
	visited.add(iPoint(19, 4));
	breadcrumbs.add(iPoint(19, 4));
	memset(cost_so_far, 0, sizeof(uint) * COST_MAP * COST_MAP);
}

void j1Pathfinding::Path(int x, int y)
{
	path.Clear();
	patfinding = false;
	iPoint goal = App->map->WorldToMap(x, y);
	int current = visited.find(goal);
	if (current != -1)
	{
		while (visited[current] != visited.start->data)
		{
			path.PushBack(visited[current]);

			goal = breadcrumbs[current];//este
			current = visited.find(goal); // este
		}
		path.PushBack(visited.start->data);
	}
	else
	{
		path.Insert(goal, 0);
	}
	//	path.PushBack(visited[current]);
	// TODO 2: Follow the breadcrumps to goal back to the origin
	// add each step into "path" dyn array (it will then draw automatically)
}

void j1Pathfinding::PropagateAstar()
{

	iPoint curr;

	if (patfinding == false)
	{

		if (frontier.Pop(curr))
		{

			int current = visited.find(curr);
			iPoint last = breadcrumbs[current];
			int distance = sqrt(pow(last.x - path[0].x, 2) + pow(last.y - path[0].y, 2));
			if (last.x != path[0].x || last.y != path[0].y)
			{
				if ((path[0].x != visited[current].x || path[0].y != visited[current].y))
				{

					iPoint neighbors[4];
					neighbors[0].create(curr.x + 1, curr.y + 0);
					neighbors[1].create(curr.x + 0, curr.y + 1);
					neighbors[2].create(curr.x - 1, curr.y + 0);
					neighbors[3].create(curr.x + 0, curr.y - 1);

					for (uint i = 0; i < 4; ++i)
					{
						int distance2 = sqrt(pow(neighbors[i].x - path[0].x, 2) + pow(neighbors[i].y - path[0].y, 2));
						if (MovementCost(neighbors[i].x, neighbors[i].y) > 0)
						{

							if (visited.find(neighbors[i]) == -1)
							{
								if (distance2 <= distance)
								{
									cost_so_far[neighbors[i].x][neighbors[i].y] = distance2;
									frontier.Push(neighbors[i], cost_so_far[neighbors[i].x][neighbors[i].y]);
									visited.add(neighbors[i]);
									breadcrumbs.add(curr);
								}

							}
						}
					}
				}
				else
				{
					patfinding = true;
				}
			}

		}
	}
}

void j1Pathfinding::PropagateDijkstra()
{

	//data.layers.At();
	// TODO 3: Taking BFS as a reference, implement the Dijkstra algorithm
	// use the 2 dimensional array "cost_so_far" to track the accumulated costs
	// on each cell (is already reset to 0 automatically)
	iPoint curr;

	if (frontier.Pop(curr))
	{

		int current = visited.find(curr);
		iPoint last = breadcrumbs[current];

		if ((path[0].x != visited[current].x || path[0].y != visited[current].y) && patfinding == false)
		{

			iPoint neighbors[4];
			neighbors[0].create(curr.x + 1, curr.y + 0);
			neighbors[1].create(curr.x + 0, curr.y + 1);
			neighbors[2].create(curr.x - 1, curr.y + 0);
			neighbors[3].create(curr.x + 0, curr.y - 1);

			for (uint i = 0; i < 4; ++i)
			{
				if (MovementCost(neighbors[i].x, neighbors[i].y) >= 0)
				{

					if (visited.find(neighbors[i]) == -1)
					{
						cost_so_far[neighbors[i].x][neighbors[i].y] = cost_so_far[last.x][last.y] + MovementCost(neighbors[i].x, neighbors[i].y);
						frontier.Push(neighbors[i], cost_so_far[neighbors[i].x][neighbors[i].y]);
						visited.add(neighbors[i]);
						breadcrumbs.add(curr);

					}
				}
			}
		}
		else
		{
			patfinding = true;
		}
	}
}



int j1Pathfinding::MovementCost(int x, int y) const
{
	int ret = -1;

	if (x >= 0 && x < App->map->data.width && y >= 0 && y < App->map->data.height)
	{
		int id = App->map->data.layers.start->next->data->Get(x, y);

		if (id == 0)
			ret = 3;
		else
			ret = 0;
	}

	return ret;
}

void j1Pathfinding::PropagateBFS()
{
	// TODO 1: Record the direction to the previous node 
	// with the new list "breadcrumps"

	iPoint curr;

	if (frontier.Pop(curr))
	{
		iPoint neighbors[4];
		neighbors[0].create(curr.x + 1, curr.y + 0);
		neighbors[1].create(curr.x + 0, curr.y + 1);
		neighbors[2].create(curr.x - 1, curr.y + 0);
		neighbors[3].create(curr.x + 0, curr.y - 1);

		for (uint i = 0; i < 4; ++i)
		{
			if (MovementCost(neighbors[i].x, neighbors[i].y) > 0)
			{
				if (visited.find(neighbors[i]) == -1)
				{
					frontier.Push(neighbors[i], 0);
					visited.add(neighbors[i]);
					breadcrumbs.add(curr);
				}
			}
		}
	}
}

void j1Pathfinding::DrawPath()
{
	iPoint point;

	// Draw visited
	p2List_item<iPoint>* item = visited.start;

	while (item)
	{
		point = item->data;
		TileSet* tileset =App->map->GetTilesetFromTileId(26);

		SDL_Rect r = tileset->GetTileRect(26);
		iPoint pos = App->map->MapToWorld(point.x, point.y);

		App->render->Blit(tileset->texture, pos.x, pos.y, &r);

		item = item->next;
	}

	// Draw frontier
	for (uint i = 0; i < frontier.Count(); ++i)
	{
		point = *(frontier.Peek(i));
		TileSet* tileset = App->map->GetTilesetFromTileId(25);

		SDL_Rect r = tileset->GetTileRect(25);
		iPoint pos = App->map->MapToWorld(point.x, point.y);

		App->render->Blit(tileset->texture, pos.x, pos.y, &r);
	}

	// Draw path
	for (uint i = 0; i < path.Count(); ++i)
	{
		iPoint pos = App->map->MapToWorld(path[i].x, path[i].y);
		App->render->Blit(tile_x, pos.x, pos.y);
	}
}