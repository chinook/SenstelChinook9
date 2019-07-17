

namespace unit
{

float getWheelRPM()
{
  static float time = 0.0f;
  time += 0.1f;

  // wheel rpm simulation acceleration
  return time;
}

float getWindSpeed()
{
  return 4.5f;
  //return 0.0f;
}

float getWinDirection()
{
  return 0.0f;
}

float getRotorRPM()
{
  return 0.0f;
}

float getTorque()
{
  return 0.0f;
}

float getLoadcell()
{
  return 0.0f;
}

}
