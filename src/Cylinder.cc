/*
 * FILE: Cylinder.cc
 * DESCRIPTION:
 * Implementation of class Cylinder.
 */
#include "Cylinder.hh"

//using namespace std;

//static constants initialization
const Vector3d Cylinder::NORMAL_TOP_CAP(0.0, -1.0, 0.0);
const Vector3d Cylinder::NORMAL_BOTTOM_CAP(0.0, 1.0, 0.0);

int Cylinder::intersect(Ray& ray, double& t, Point3d& P, Vector3d& N)
{
  //Reduced points are projections of original point to the plane Y=0
  Point2d reducedD(ray.D()[0], ray.D()[2]);
  Point2d reducedO_C(ray.O()[0] - _bottomCenter[0],
                     ray.O()[2] - _bottomCenter[2]);
  /* Ray intersection with the cylinder body is an equation:
   * (Ray(t)[x] - center[x])^2 + (Ray(t)[z] - center[z])^2 - radius^2 == 0
   * Ray(t) = rayO + rayD * t, and we get quadratic equation by t:
   * at^2 + bt + c == 0
   * Originally b estimated as
   * b = 2 * [ rayD[x]*(rayO[x] - Center[x]) + rayD[z]*(rayO[z] - Center[z]) ];
   * Instead of solving [-b +- sqrt(b^2 - 4ac) ] / 2a
   * let us reduce b by 2 and get
   * [-b' +- sqrt(b'^2 - ac) ] / a
   * there b' = rayD[x]*(rayO[x] - Center[x]) + rayD[z]*(rayO[z] - Center[z]);
   */

  double a = reducedD.sqrnorm();

  //special case: the ray is parallel to Y axis
  if (approxEqual(a, 0.0)) return parallelRay(ray, t, P, N);


  double b = reducedD | reducedO_C;
  double c = reducedO_C.sqrnorm() - _sqrRadius;
  double d = sqr(b) - a * c;

  if (d < 0.0) return 0; //no roots

  d = sqrt(d);
  a = 1.0 / a;

  /* Candidate t values in intersections:
   * 0,1 - intersections with the cylinder body;
   * 2,3 - intersections with bottom and top caps respectively;
   */
  double candidateT[4] = {0.0, 0.0, 0.0, 0.0};

  double y0, y1; //Y coordinates of t0 and t1

  t = (-b - d) * a; //first equation root
  P = ray(t);
  y0 = P[1];
  if (t > EPS && y0 + EPS > _bottomCenter[1] && y0 - EPS < _topCenter[1])
  { //add candidate t only if it in the cylinder range
    candidateT[0] = t;
  }

  t = (-b + d) * a; //second equation root
  P = ray(t);
  y1 = P[1];
  if (t > EPS && y1 + EPS > _bottomCenter[1] && y1 - EPS < _topCenter[1])
  { //add candidate t only if it in the cylinder range
    candidateT[1] = t;
  }

  //ensure what y0 <= y1
  if (y0 > y1)
  {
    y1 = y0;
    y0 = P[1];
  }
  assert(y0 <= y1);

  if (!approxEqual(ray.D()[1], 0.0))
  { //check intersections with caps only if ray is not parallel to Y=0 plane
    //estimating candidate t from distance on Y coordinate

    if (y0 - EPS < _bottomCenter[1] && y1 + EPS > _bottomCenter[1])
    { //the ray intersects with the bottom cap
      candidateT[2] = (_bottomCenter[1] - ray.O()[1]) / ray.D()[1];
    }
    if (y0 - EPS < _topCenter[1] && y1 + EPS > _topCenter[1])
    { //the ray intersects with the top cap
      candidateT[3] = (_topCenter[1] - ray.O()[1]) / ray.D()[1];
    }
  }


  t = INF;
  int choise = -1; //index of the chosen candidate

  //take the minimal positive t from the candidates
  for (int i = 0; i < 4; ++i)
  {
    if (candidateT[i] < t && candidateT[i] > EPS)
    {
      t = candidateT[i];
      choise = i;
    }
  }

  //no intersections: all candidates were bad
  if (choise == -1) return 0;

  //the real intersection point
  P = ray(t);

  //computing the normal
  switch (choise)
  {
  case 0:
  case 1:
    //normal to the cylinder body
    N = P - _bottomCenter;
    N[1] = 0.0;
    N *= _invRadius; //normalizing
    break;
  case 2:
    //normal to the bottom cap
    N = NORMAL_TOP_CAP;
    break;
  case 3:
    //normal to the top cap
    N = NORMAL_BOTTOM_CAP;
    break;
  default:
    break;
  }

  return 1;
}

Color3d Cylinder::texture_diffuse(const Point3d &P)
{
  //no texture map or zero-radius sphere
  if (_diffuse_texture == NULL || _radius < EPS) return COLOR_WHITE;

  //translate the cylinder bottom center to be in origin
  const Point3d cartesian = P - _bottomCenter;

  /* As defined by the cylindrical coordinates:
   * u = 0.5 + atan(x/z) / 2pi
   * v = y / height
   */
  TexCoord pos(atan2(cartesian[0], cartesian[2]), cartesian[1] * _invHeight);
  pos[0] = pos[0] * INV_TWO_PI + 0.5;

  clipCoords(pos);
  return map_texture(pos);
}
