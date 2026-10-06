# rorbot no 20
import abc
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

def BASIC_DE_FUZ(f,output_S,output_D):
   sum_frontside = 0
   sum_frontside2 = 0
   summation_n = 0
   summation_n2 = 0
   output = 0
   output2 = 0

#rule=z
#strength=y
#all fuzzy rules 
   for z,y in f.items():
       if y > 0:     # If the fuzzy value is positive
          
           if (z[0]=="near") & (z[1]=="near"):
               summation_n += output_S["slow"][1]*y
               summation_n2 += output_D["left"][1]*y
           if (z[0]=="near") & (z[1]=="medium"):
               summation_n += output_S["slow"][1]*y
               summation_n2 += output_D["left"][1]*y
           if (z[0]=="near") & (z[1]=="far"):
               summation_n += output_S["slow"][1]*y
               summation_n2 += output_D["left"][1]*y
           if (z[0]=="medium") & (z[1]=="near"):
               summation_n += output_S["slow"][1]*y
               summation_n2 += output_D["left"][1]*y
           if (z[0]=="medium") & (z[1]=="medium"):
               summation_n += output_S["slow"][1]*y
               summation_n2 += output_D["zero"][1]*y
           if (z[0]=="medium") & (z[1]=="far"):
               summation_n += output_S["slow"][1]*y
               summation_n2 += output_D["left"][1]*y
           if (z[0]=="far") & (z[1]=="near"):
               summation_n += output_S["slow"][1]*y
               summation_n2 += output_D["right"][1]*y
           if (z[0]=="far") & (z[1]=="medium"):
               summation_n += output_S["slow"][1]*y
               summation_n2 += output_D["right"][1]*y
           if (z[0]=="far") & (z[1]=="far"):
               summation_n += output_S["medium"][1]*y
               summation_n2 += output_D["right"][1]*y
           sum_frontside += y    
           sum_frontside2 += y    

# Defuzzification step
       if(sum_frontside > 0):
           output  = summation_n/sum_frontside    
       if(sum_frontside2 > 0):
           output2 = summation_n2/sum_frontside2     

# Return the calculated linear and angular velocities
   a = {"linear": output, "angular": output2,}
   return a

# Sensor ranges for frontward and backward directions
sensor_F = {"near": [0,0.6,0.65],"medium": [0.6,0.65,0.7],"far": [0.65,0.7,1]}
sensor_B =  {"near": [0,0.6,0.65],"medium": [0.6,0.65,0.7],"far": [0.65,0.7,1]}

# Output speeds and directions for fuzzy rules
output_S = {"slow": [0.01,0.05,0.1], "medium": [0.1,0.15,0.2], "fast": [0.2,0.25,0.3]}
output_D = {"right": [0.1,0.5,0.3],"zero": [-1,0,1],"left": [-0.3,-0.2,-0.1]}

# Function for fuzzy inference
def BASIC_FUZ(ROBOT_REG,sensor_F,sensor_B):

   f = {
       ("near","near"):0,("near","medium"):0, ("near","far"):0, ("medium","near"): 0, 
       ("medium","medium"):0, ("medium","far"):0, 
       ("far","near"):0, ("far","medium"):0, ("far","far"): 0,
       }

 # Initialize fuzzy variables for front and back sensors
 # RFS= Right Front Sensor
 # RBS= Right Back Sensor
   RFS_near = 0
   RFS_medium = 0
   RFS_far = 0
   RBS_near = 0
   RBS_medium = 0
   RBS_far = 0

   for z,y in ROBOT_REG.items():
       if z == "fright":    
       
       #category=a
       #range_vals=b
           for a,b in sensor_F.items():

               if (y >= b[0]) & (y <= b[2]):    

                   if a == "near":      
                       if y <= b[1]:    
                           RFS_near = (y-b[0])/(b[1]-b[0])        
                       else:
                           RFS_near = (b[2]-y)/(b[2]-b[0])      

                   elif a == "medium":         
                       if y <= b[1]:
                           RFS_medium = (y-b[0])/(b[1]-b[0])     
                       else:
                           RFS_medium = (b[2]-y)/(b[2]-b[0])    
                   elif a == "far":      
                       if y == 1:
                           RFS_far = 1    
                       elif y <= b[1]:
                           RFS_far = (y-b[0])/(b[1]-b[0])     
                       else:  
                           RFS_far = (b[2]-y)/(b[2]-b[0])


       if z == "bright":     
           for a,b in sensor_B.items():

               if (y >= b[0]) & (y <= b[2]):     
                   if a == "near":    
                       if y <= b[1]:
                           RBS_near = (y-b[0])/(b[1]-b[0])   
                       else:
                           RBS_near = (b[2]-y)/(b[2]-b[0])
                   elif a == "medium":      
                       if y <= b[1]:
                           RBS_medium = (y-b[0])/(b[1]-b[0])     
                       else:
                           RBS_medium = (b[2]-y)/(b[2]-b[0])
                   elif a == "far":     
                       if y == 1:
                           RBS_far = 1    
                       elif y <= b[1]:
                           RBS_far = (y-b[0])/(b[1]-b[0])    
                       else:  
                           RBS_far = (b[2]-y)/(b[2]-b[0])

 # Fuzzy rules based on sensor data
   if (RFS_near>0) & (RBS_near>0):
       m = min(RFS_near,RBS_near)
       f[("near","near")] = m
   if (RFS_near>0) & (RBS_medium>0):
       m = min(RFS_near,RBS_medium)
       f[("near","medium")] = m
   if (RFS_near>0) & (RBS_far>0):
       m = min(RFS_near,RBS_far)
       f[("near","far")] = m
   if (RFS_medium>0) & (RBS_near>0):
       m = min(RFS_medium,RBS_near)
       f[("medium","near")] = m
   if (RFS_medium>0) & (RBS_medium>0):
       m = min(RFS_medium,RBS_medium)
       f[("medium","medium")] = m
   if (RFS_medium>0) & (RBS_far>0):
       m = min(RFS_medium,RBS_far)
       f[("medium","far")] = m
   if (RFS_far>0) & (RBS_near>0):
       m = min(RFS_far,RBS_near)
       f[("far","near")] = m
   if (RFS_far>0) & (RBS_medium>0):
       m = min(RFS_far,RBS_medium)
       f[("far","medium")] = m
   if (RFS_far>0) & (RBS_far>0):
       m = min(RFS_far,RBS_far)
       f[("far","far")] = m
   return f     # Return the computed fuzzy values

mynode_ = None
pub_ = None

regions_ = {
   'fright': 0, 
   'bright': 0, 
}

twstmsg_ = None

# Timer callback function 
def timer_callback():
   global pub_, twstmsg_
   if ( twstmsg_ != None ):
       pub_.publish(twstmsg_)
       
# Callback function for laser scan data
def clbk_laser(msg):
   global regions_, twstmsg_
   regions_ = {
       'bright': find_nearest(msg.ranges[200:220]),     
       'fright': find_nearest(msg.ranges[320:340]),     
   }
   twstmsg_= movement()      

# Function to find the nearest non-zero distance
def find_nearest(list):
   f_list = filter(lambda item: item > 0.0, list)  
   return min(min(f_list, default=1), 1)    

# Function to calculate robot's movement based on fuzzy logic outputs
def movement():
   global regions_,mynode_,z,e,sensor_F,sensor_B,output_S,output_D
   ROBOT_REG = regions_      

   print('fright', regions_['fright'])
   print('bright', regions_['bright'])

   msg = Twist()           
   
 # Compute fuzzy outputs
   f = BASIC_FUZ(ROBOT_REG,sensor_F,sensor_B)

# Print linear and angular velocity outputs from fuzzy logic
   print(("BASIC_DE_FUZ(f,output_S,output_D)[])",BASIC_DE_FUZ(f,output_S,output_D)["linear"]))
   print("BASIC_DE_FUZ angular ",  BASIC_DE_FUZ(f,output_S,output_D)["angular"])
   
   msg.linear.x = (BASIC_DE_FUZ(f,output_S,output_D)["linear"])
   msg.angular.z = -BASIC_DE_FUZ(f,output_S,output_D)["angular"]      

   return msg      

def stop():
   global pub_
   msg = Twist()     
   msg.angular.z = 0.0
   msg.linear.x = 0.0
   pub_.publish(msg)      

def main():
   global pub_, mynode_
   rclpy.init()
   mynode_ = rclpy.create_node('reading_laser')
   
   qos = QoSProfile(
       depth=10,
       reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
   )
   pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)
   sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)
   timer_period = 0.2  # Timer period for publishing velocity commands (every 0.2 seconds)
   timer = mynode_.create_timer(timer_period, timer_callback)

   try:
       rclpy.spin(mynode_)
   except KeyboardInterrupt:
       stop()  
   except:
       stop() 
        
   finally:
       mynode_.destroy_timer(timer)

       mynode_.destroy_node()

       rclpy.shutdown()

if __name__ == '__main__':

   main()
