import math
import argparse

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
  return A1, A2
  # return deg(A1), deg(A2)

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


parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('integers', metavar='N', type=int, nargs='+',
                    help='an integer for the accumulator')
# parser.add_argument('--sum', dest='accumulate', action='store_const',
#                     const=sum, default=max,
#                     help='sum the integers (default: find the max)')

args = parser.parse_args()
# print(args.integers)
# print(args.integers[0])

# print(angles(10,0,10,10))
# print(angles(5,0,10,10))
# a,b = angles(5,0,4,5)
a,b = angles(args.integers[0],args.integers[1],args.integers[2],args.integers[3])
# print(a,b) 
print(math.pi/2-a,math.pi-b) 
# print(angles(4,8,4,5))

