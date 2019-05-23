import math

def lawOfCosines(a, b, c):
  return math.acos( (a*a + b*b - c*c) / (2 * a * b) )

def distance(x, y):
  return math.sqrt(x*x + y*y)

def deg(rad):
  return rad * 180 / math.pi

def angles(x, y, len1, len2):
  dist = distance(x, y)
  D1 = math.atan2(y, x)
  D2 = lawOfCosines(dist, len1, len2)
  A1 = D1 + D2
  A2 = lawOfCosines(len1, len2, dist)
  return deg(A1), deg(A2)

#   y
#   |             -
#   |            /     -
#   |           /  A2       -  len2
#   |          /                 -
#   |         /                       -
#   |        /len1                         . (x,y)
#   |       /                         .
#   |      /                    .
#   |     /                .
#   |    /  D2        .      dist
#   |   /        .
#   |  /    .
#   | /. A1          D1
#   (0,0)---------------------------------------x

print(angles(10,0,10,10))
print(angles(5,0,10,10))

