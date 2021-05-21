
#include "my_math.h"
#include <vector>

struct QuadTree
{
    QuadTree *quadrants[4] = {};

    static const int MAX_POINTS = 1;
    int num_points = 0;
    v2 *points[MAX_POINTS] = {};

    v2 bl = {};
    v2 tr = {};

    QuadTree(v2 in_bl, v2 in_tr) : bl(in_bl), tr(in_tr) {}

    QuadTree *insert(v2 *point)
    {
        if(num_points >= MAX_POINTS)
        {
            if(quadrants[0] == nullptr)
            {
                v2 tl = {bl.x, tr.y};
                v2 br = {tr.x, bl.y};
                quadrants[0] = new QuadTree((bl + tr) / 2.0f, tr);
                quadrants[1] = new QuadTree((bl + tl) / 2.0f, (tl + tr) / 2.0f);
                quadrants[2] = new QuadTree(bl, (bl + tr) / 2.0f);
                quadrants[3] = new QuadTree((bl + br) / 2.0f, (br + tr) / 2.0f);
            }

            v2 local_point = *point - ((bl + tr) / 2.0f);
            if(local_point.y > 0.0f)
            {
                if(local_point.x > 0.0f)
                {
                    return quadrants[0]->insert(point);
                }
                else
                {
                    return quadrants[1]->insert(point);
                }
            }
            else
            {
                if(local_point.x > 0.0f)
                {
                    return quadrants[3]->insert(point);
                }
                else
                {
                    return quadrants[2]->insert(point);
                }
            }
        }
        else
        {
            points[num_points++] = point;
            return this;
        }

        
    }

    void get_in_range(std::vector<v2 *> *results, v2 other_bl, v2 other_tr)
    {
        for(int i = 0; i < num_points; i++)
        {
            v2 *p = points[i];
            if(p->x > other_bl.x && p->x < other_tr.x &&
               p->y > other_bl.y && p->y < other_tr.y)
            {
                results->push_back(p);
            }
        }

        if(quadrants[0] != nullptr)
        {
            for(int i = 0; i < 4; i++)
            {
                if(tr.x < other_bl.x || bl.x > other_tr.x ||
                   tr.y < other_bl.y || bl.y > other_tr.y)
                {
                    continue;
                }
                else
                {
                    quadrants[i]->get_in_range(results, other_bl, other_tr);
                }
            }
        }
    }

    int debug_count()
    {
        int n = 0;
        n += num_points;
        if(quadrants[0] != nullptr)
        {
            n += quadrants[0]->debug_count();
            n += quadrants[1]->debug_count();
            n += quadrants[2]->debug_count();
            n += quadrants[3]->debug_count();
        }
        return n;
    }

};

