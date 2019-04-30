////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: distance from tip, 0.18m

//coordinates where the cut should be
float fix1, fix2;
float cutCoord[3] = {0.0};
float foo = 0;
cutCoord[0] = cloud->points[idx1].x + vec1(0)*foo;
cutCoord[1] = cloud->points[idx1].y + vec1(1)*foo;
cutCoord[2] = cloud->points[idx1].z + vec1(2)*foo;

for (int i = 0; i < 3; i++)
{
    cout << "cutCoord ["  << i << "]: " << cutCoord[i] << endl;
}

//nearest point on one side
float cutWidth = 0;
for (int point = 0; point < cloud->points.size()/2; point++)
{
    xdist = cutCoord[0] - cloud->points[point].x;
    ydist = cutCoord[1] - cloud->points[point].y;
    zdist = cutCoord[2] - cloud->points[point].z;
    len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));
    cout << "x = " << xdist << " y = " << ydist << " z = " << zdist << endl;
    cout << "length = " << len << endl;
    if (len>cutWidth)
    {
        fix1 = point;
        cout << "fix 1 = " << fix1 << endl;
    }
}

double distx = cloud->points[idx1].x - cloud->points[idx2].x;
double disty = cloud->points[idx1].y - cloud->points[idx2].y;
double distz = cloud->points[idx1].z - cloud->points[idx2].z;
//Finding the length of the line
double len1 = sqrt(pow(distx, 2.0) + pow(disty, 2.0) + pow(distz, 2.0));
cout << "old length = " << len1 << endl;
double difflen = 0.14/len1;
cout << "differnece = " << difflen << endl;
//Scaling the line to 0.18
len1 = len1 * difflen;
cout << "New length = " << len1 << endl;

cout << " x = " << distx << "  y = " << disty << "  z = " << distz << endl;

//Scaling the distances -> It seems to be a stupid way to do this and thus should probably just be deleted
distx = distx * difflen;
disty = disty * difflen;
distz = distz * difflen;

cout << "new x = " << distx << " new y = " << disty << " new z = " << distz << endl;

//By modifying t we can find any point on the line
//where t = 0 corresponds to our starting point and t = 1 to our end point
//any value of t in between will be a point on the line between start and end
double t = -1 * difflen;
//Here we calculate the direction "vector"
double directionx = cloud->points[idx1].x - cloud->points[idx2].x;
double directiony = cloud->points[idx1].y - cloud->points[idx2].y;
double directionz = cloud->points[idx1].z - cloud->points[idx2].z;


//Pushing the point on the line 18cm from start to the pointcloud
//Notice that each point is calculated by adding the start point with the direction vector multiplied by t
pcl::PointXYZ pp;
pp.x = cloud->points[idx1].x + directionx * t; pp.y = cloud->points[idx1].y + directiony * t; pp.z = cloud->points[idx1].z + directionz * t;
cloud->push_back(pp);

cout << cloud->points.size() << endl;
//Here we find the distance from the start of the line to the point
int lastindex = cloud->points.size()-1;
double distpointx = cloud->points[idx1].x - cloud->points[lastindex].x;
double distpointy = cloud->points[idx1].y - cloud->points[lastindex].y;
double distpointz = cloud->points[idx1].z - cloud->points[lastindex].z;
double lenToPoint = sqrt(pow(distpointx, 2.0) + pow(distpointy, 2.0) + pow(distpointz, 2.0));
double lenToPoint1 = lenToPoint;
//Here we search for the smallest distance to the 18cm point from any point in the pointclooud
//The shortst distance to the point will then be the last i value -1 since the smallest distance to the point is the distance from the point to the itself

for(int i = 0; i < cloud->points.size(); i++)
{
	double itepointx = cloud->points[i].x - cloud->points[lastindex].x;
	double itepointy = cloud->points[i].y - cloud->points[lastindex].y;
	double itepointz = cloud->points[i].z - cloud->points[lastindex].z;

	if(lenToPoint >= sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0)))
		{
			lenToPoint = sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0));
			cout << " i = " << i << endl;

		//cout << "i = " << i << "x = " << minx << " y = " << miny << " z = " << minz << endl;
		}
}
cout << "g" << endl;
for(int i = 0; i < cloud->points.size()/2; i++)
{
	double itepointx = cloud->points[i].x - cloud->points[lastindex].x;
	double itepointy = cloud->points[i].y - cloud->points[lastindex].y;
	double itepointz = cloud->points[i].z - cloud->points[lastindex].z;

	if(lenToPoint1 > sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0)))
		{
			lenToPoint1 = sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0));
			cout << " i1 = " << i << endl;

		//cout << "i = " << i << "x = " << minx << " y = " << miny << " z = " << minz << endl;
}
}
//nearest point to that point, on the other side
cutWidth = 1.0;
for (int point = 0; point < cloud->points.size(); point++)
{
    if (abs(point-fix1)>40) //exclude immediate neighbors
    {
        xdist = cloud->points[fix1].x - cloud->points[point].x;
        ydist = cloud->points[fix1].y - cloud->points[point].y;
        zdist = cloud->points[fix1].z - cloud->points[point].z;
        len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

        if (len<cutWidth)
        {
            cutWidth = len;
            fix2 = point;
        }
    }
}
