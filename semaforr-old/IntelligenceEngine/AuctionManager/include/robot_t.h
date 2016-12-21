#ifndef ROBOT_T_H
#define ROBOT_T_H

#include "defines.h"

class robot_t {
    public:
        int id;
        point_t p;
        bundle_t goals;
        robot_t(int id, int x, int y) : id(id) { p.first = x; p.second = y; }

        static double euclidian(point_t a, point_t b)
        {
            double dx = (double)a.first - (double)b.first;
            double dy = (double)a.second - (double)b.second;
            return sqrt(dx*dx + dy*dy);
        }

        static point_t tri_centroid(point_t a, point_t b, point_t c)
        {
            point_t centroid;
            centroid.first  =  (1.0 / 3.0)*(a.first  + b.first  + c.first);
            centroid.second =  (1.0 / 3.0)*(a.second + b.second + c.second);
            return centroid;
        }

        double get_total_path_length()
        {
            if(goals.empty())
                return 0;

            double total = euclidian(this->p, goals[0]);
            for(size_t i = 1; i < goals.size(); i++)
            {
                total += euclidian(goals[i-1], goals[i]);
            }
            return total;
        }

        double get_total_path_length(bundle_t bundle)
        {
            double  total = euclidian(this->p, bundle[0]);

            for(size_t i = 1; i < bundle.size(); i++)
                total += euclidian(bundle[i-1], bundle[i]);
            return total;
        }

        string get_bid(auction_t auc)
        {
            double minbid = this->get_total_path_length(auc.get_bundle(0));
            size_t mindex = 0;
            for(int i = 1; i < auc.bundle_size(); i++)
            {
                double tmp = get_total_path_length(auc.get_bundle(i));
                if(minbid > tmp)
                {
                    minbid = tmp;
                    mindex = i;
                }
            }
            sstream bidmess;
            bidmess << "AUCTION_BID " << auc.get_id() << " " << this->id << " " << mindex << " " << minbid;
            return bidmess.str();
        }

};

#endif

