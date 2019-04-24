#include <iostream>
#include <vector>

using namespace std;

vector <int> intersectingSegments(int longa, int longb, int shorta[], int shortb[], int size)
{
    vector <int> foundSegments;
    for (int i = 0; i < size; i++)
    {
        float A1 = (cloud->points[longa].y - cloud->points[longb].y) / (cloud->points[longa].x - cloud->points[longb].x);
        float A2 = (cloud->points[shorta[i]].y - cloud->points[shortb[i]].y) / (cloud->points[shorta[i]].x - cloud->points[shortb[i]].x);
        float b1 = cloud->points[longa].y - A1 * cloud->points[longa].x;
        float b2 = cloud->points[shorta[i]].y - A2 * cloud->points[shorta[i]].x;

        if (A1 == A2) {continue;} //parallel segments

        float Xa = (b2 - b1) / (A1 - A2);

        if (Xa < max(min(cloud->points[longa].x, cloud->points[longb].x), min(cloud->points[shorta[i]].x), cloud->points[shortb[i]].x) || (Xa > min(max(cloud->points[longa].x, cloud->points[longb].x), max(cloud->points[shorta[i]].x), cloud->points[shortb[i]].x)))
            {continue;} // Intersection out of bound
        else {foundSegments.push_back(shorta[i]); foundSegments.push_back(shortb[i]);} //Every second element beginning/end of the line
    }
    return foundSegments;
}

int main()
{
    return 0;
}
