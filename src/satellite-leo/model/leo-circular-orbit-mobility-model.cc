/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Tim Schubert <ns-3-leo@timschubert.net>
 */

#include "math.h"

#include "ns3/double.h"
#include "ns3/simulator.h"

#include "leo-circular-orbit-mobility-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LeoCircularOrbitMobilityModel");

NS_OBJECT_ENSURE_REGISTERED (LeoCircularOrbitMobilityModel);

TypeId
LeoCircularOrbitMobilityModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::LeoCircularOrbitMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Leo")
    .AddConstructor<LeoCircularOrbitMobilityModel> ()
    .AddAttribute ("Altitude",
                   "A height from the earth's surface in kilometers",
                   DoubleValue (1000.0),
                   MakeDoubleAccessor (&LeoCircularOrbitMobilityModel::SetAltitude,
                   		       &LeoCircularOrbitMobilityModel::GetAltitude),
                   MakeDoubleChecker<double> ())
    // TODO check value limits
    .AddAttribute ("Inclination",
                   "The inclination of the orbital plane in degrees",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&LeoCircularOrbitMobilityModel::SetInclination,
                   		       &LeoCircularOrbitMobilityModel::GetInclination),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Precision",
                   "The time precision with which to compute position updates. 0 means arbitrary precision",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&LeoCircularOrbitMobilityModel::m_precision),
                   MakeTimeChecker ())
    ;
  return tid;
}

LeoCircularOrbitMobilityModel::LeoCircularOrbitMobilityModel() : MobilityModel (), m_longitude (0.0), m_offset (0.0), m_position (),flagLeo(false)
{
  NS_LOG_FUNCTION_NOARGS ();
}

LeoCircularOrbitMobilityModel::~LeoCircularOrbitMobilityModel()
{
}

Vector3D
CrossProduct (const Vector3D &l, const Vector3D &r)
{
  return Vector3D (l.y * r.z - l.z * r.y,
		   l.z * r.x - l.x * r.z,
		   l.x * r.y - l.y * r.x);
}

Vector3D
Product (const double &l, const Vector3D &r)
{
  return Vector3D (l * r.x,
		   l * r.y,
		   l * r.z);
}

double
DotProduct (const Vector3D &l, const Vector3D &r) //向量内积
{
  return (l.x* r.x) + (l.y*r.y) + (l.z*r.z);
}

double
LeoCircularOrbitMobilityModel::GetSpeed () const
{
  return sqrt (LEO_EARTH_GM_KM_E10 / m_orbitHeight) * 1e5; //由　向心力＝万有引力　得到　v = sqrt(GM/R) 
}

Vector
LeoCircularOrbitMobilityModel::DoGetVelocity () const
{
  // Vector3D pos = DoGetPosition ();
  // pos = Vector3D (pos.x / pos.GetLength (), pos.y / pos.GetLength (), pos.z / pos.GetLength ());
  // Vector3D heading = CrossProduct (PlaneNorm (), pos);
  // return Product (GetSpeed (), heading);
  return Vector(0,0,0);
}

Vector3D
LeoCircularOrbitMobilityModel::PlaneNorm () const
{
  double lat = CalcLongitude ();
  return Vector3D (sin (-m_inclination) * cos (lat),
  		   sin (-m_inclination) * sin (lat),
  		   cos (m_inclination));
}

double
LeoCircularOrbitMobilityModel::GetProgress (Time t) const
{
  // TODO use nanos or ms instead? does it give higher precision?
  int sign = 1;
  // ensure correct gradient (not against earth rotation)
  if (m_inclination > M_PI/2)
    {
      sign = -1;
    }
  // 2pi * (distance travelled / circumference of earth) + offset
  return sign * (((GetSpeed () * t.GetSeconds ()) / (LEO_EARTH_RAD_KM * 1000))) + m_offset;
}

Vector3D
LeoCircularOrbitMobilityModel::RotatePlane (double a, const Vector3D &x) const
{
  Vector3D n = PlaneNorm ();

  return Product (DotProduct (n, x), n)
    + Product (cos (a), CrossProduct (CrossProduct (n, x), n))
    + Product (sin (a), CrossProduct (n, x));
}

// double
// LeoCircularOrbitMobilityModel::CalcLatitude () const
// {
//   return m_longitude + ((Simulator::Now ().GetDouble () / Hours (24).GetDouble ()) * 2 * M_PI);
// }

double
LeoCircularOrbitMobilityModel::CalcLongitude () const
{
  return m_longitude + ((Simulator::Now ().GetDouble () / Hours (24).GetDouble ()) * 2 * M_PI);
}

// Vector
// LeoCircularOrbitMobilityModel::CalcPosition (Time t) const
// //需要改这里的代码，卫星第一次确定自己位置也是在这里确定的，后续更改自己位置也是在这里
// {

//   double lat = CalcLongitude ();
//   // account for orbit latitude and earth rotation offset
//   Vector3D x = Product (m_orbitHeight*1000, Vector3D (cos (m_inclination) * cos (lat),
//               cos (m_inclination) * sin (lat),
//               sin (m_inclination)));
//   //x计算的是轨道初始位置，即第一个卫星的位置，下面计算Angle后得到偏移量（假如是3颗卫星，则每颗卫星偏移1/3），RotatePlane计算这颗卫星在轨道中的位置
//   double Angle = GetProgress(t);  //这里的angle已经进行了pi/180运算
//   return RotatePlane (Angle, x);

// }

Vector //修改版 by ljy
LeoCircularOrbitMobilityModel::CalcPosition (Time t) const
//需要改这里的代码，卫星第一次确定自己位置也是在这里确定的，后续更改自己位置也是在这里
{
  if(!flagLeo)
  {
    double lon = CalcLongitude ();
    double incl = 10*M_PI/180;
    // account for orbit latitude and earth rotation offset
    Vector3D x = Product (m_orbitHeight*1000, Vector3D (cos (incl) * cos (lon),
                cos (incl) * sin (lon),
                sin (incl)));
    //x计算的是轨道初始位置，即第一个卫星的位置，下面计算Angle后得到偏移量（假如是3颗卫星，则每颗卫星偏移1/3），RotatePlane计算这颗卫星在轨道中的位置
    double Angle = GetProgress(t);  //这里的angle已经进行了pi/180运算
    Vector3D m_position_temp = RotatePlane (Angle, x);
    flagLeo = true;
    m_lat = CalcLat(m_position_temp);
    m_lon = CalcLon(m_position_temp);
    return m_position_temp;
  }
  else
  {
    Vector3D m_position_temp = nextPrecisionPosition();
    return m_position_temp;
  }
}


Vector3D
LeoCircularOrbitMobilityModel::nextPrecisionPosition() const
{
  int T = 2*M_PI*sqrt(pow(m_orbitHeight,3)/LEO_EARTH);
  double incl = GetInclination ();
  double Precision_lat = double((4*incl)/T);
  double Precision_lon = double(360.0/T);
  NS_LOG_DEBUG(Precision_lat << " " << Precision_lon);

  //出bug的轮次  y: 1.10352e-06 Lat: -10 Lon 180
  //->          y: -7145.86 Lat: -9.9939 Lon -179.945
  if(m_position.y >= 0)
  {
    m_lat = m_lat - Precision_lat;
    if(m_lat <= -10)
    {
      m_lat = -(m_lat + incl*2);
    }

    m_lon = m_lon + Precision_lon;
    if(m_lon >= 180)
    {
      m_lon = m_lon - 360;
    }
  }
  else if(m_position.y < 0)
  {
    m_lat = m_lat + Precision_lat;
    if(m_lat >= 10)
    {
      m_lat = incl*2 - m_lat;
    }

    m_lon = m_lon + Precision_lon;

  }
  
  Vector3D newPos = GetPositionByLL(m_lat,m_lon,m_orbitHeight);
  return newPos;

}

Vector3D 
LeoCircularOrbitMobilityModel::GetPositionByLL(double lat, double lon, double height) const
{
  double angle_lat = lat * (M_PI / 180);
  double angle_lon = lon * (M_PI / 180);
  // Vector3D pos = Vector3D (LEO_GND_RAD_EARTH * sin (lat) * cos (lon),
  // 			   LEO_GND_RAD_EARTH * sin (lat) * sin (lon),
  // 			   LEO_GND_RAD_EARTH * cos (lat));

  //修改版
  //以北纬为正，以东经为正　　（即处在北纬0-90°＋东经0-90°的视为八个象限中的第一象限）
  Vector3D pos = Vector3D(height*1000 * cos (angle_lat) * cos (angle_lon),
  			   height*1000  * cos (angle_lat) * sin (angle_lon),
  			   height*1000  * sin (angle_lat));

  return pos;
}

double 
LeoCircularOrbitMobilityModel::CalcLat(Vector pos) const
{
  double LatSource = atan(pos.z/(sqrt(pos.x*pos.x + pos.y*pos.y)))/(M_PI / 180);
  return LatSource;
}
double 
LeoCircularOrbitMobilityModel::CalcLon(Vector pos) const
{
  double LonSource = atan(pos.y / pos.x)/(M_PI / 180);
  if(pos.x<0 && pos.y>0) LonSource+=180;
  if(pos.x<0 && pos.y<0) LonSource-=180;
  return LonSource;
}

Vector LeoCircularOrbitMobilityModel::Update ()
{
  m_position = CalcPosition (Simulator::Now ());
  NotifyCourseChange ();

  if (m_precision > Seconds (0))
    {
      Simulator::Schedule (m_precision, &LeoCircularOrbitMobilityModel::Update, this);
    }

  return m_position;
}

Vector
LeoCircularOrbitMobilityModel::DoGetPosition (void) const
{
  if (m_precision == Time (0))
    {
      // Notice: NotifyCourseChange () will not be called
      return CalcPosition (Simulator::Now ());
    }
  return m_position;
}

void
LeoCircularOrbitMobilityModel::DoSetPosition (const Vector &position)
{
  // use first element of position vector as latitude, second for longitude
  // this works nicely with MobilityHelper and GetPostion will still get the
  // correct position, but be aware that it will not be the same as supplied to
  // SetPostion
  m_longitude = position.x;
  m_offset = position.y;

  Update ();
}

double LeoCircularOrbitMobilityModel::GetAltitude () const
{
  return m_orbitHeight - LEO_EARTH_RAD_KM;
}

void LeoCircularOrbitMobilityModel::SetAltitude (double h)
{
  m_orbitHeight = LEO_EARTH_RAD_KM + h;
  // Update ();
}

double LeoCircularOrbitMobilityModel::GetInclination () const
{
  // return (m_inclination / M_PI) * 180.0;
  return (m_inclination / M_PI) * 180.0;
   //因为北纬南纬最多只有90°，所以这里要改成除以90 
}

void LeoCircularOrbitMobilityModel::SetInclination (double incl)
{
  NS_ASSERT_MSG (incl != 0.0, "Plane must not be orthogonal to axis");
  // m_inclination = (incl / 180.0) * M_PI;
  m_inclination = (incl / 180.0) * M_PI; //同理改成90
  //by ljy 2024/04/04
  // Update ();

}

};
