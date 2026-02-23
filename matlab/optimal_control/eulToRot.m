%w_R_b
function R = eulToRot(rpy)

roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);

Rx =[	   1   ,    0     	  ,  	  0,
           0   ,    cos(roll) ,  -sin(roll),
           0   ,    sin(roll),  cos(roll)];
       

Ry =[ cos(pitch) 	,	 0  ,   sin(pitch),
            0       ,    1  ,   0,
      -sin(pitch) 	,	0   ,  cos(pitch)];
  

Rz =[ cos(yaw)  ,  -sin(yaw) ,		0,
      sin(yaw) ,  cos(yaw) ,  		0,
         0      ,     0     ,       1];


R =  Rz*Ry*Rx;
end

