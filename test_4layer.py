from DEMSystem import DEMSystem
import os
import math

# You should change 'test' to your preferred folder.
MYDIR = ("fld_4layer_gt")
MYDIR1 = ("col")
CHECK_FOLDER = os.path.isdir(MYDIR)
CHECK_FOLDER1 = os.path.isdir(MYDIR1)

# If folder doesn't exist, then create it.
if not CHECK_FOLDER:
    os.makedirs(MYDIR)
    print("created folder : ", MYDIR)
else:
    print(MYDIR, "folder already exists.")
    
if not CHECK_FOLDER1:
    os.makedirs(MYDIR1)
    print("created folder : ", MYDIR1)
else:
    print(MYDIR1, "folder already exists.")



domain_x = 6
domain_y = 6
domain_z = 3
radius = 0.2
tol = 0.05
pos_list = []
vel_list = []
fix_list = []

num = int((domain_x-2) / (2 * radius + tol))
for k in range (0,4,1):
    for j in range(0, num, 1):
        for i in range(0, num, 1):
            pos = []
            if k%2 == 0:
                pos.append(-domain_x/2+1+i*(radius*2+tol))
            else:
                pos.append(-domain_x/2+1+i*(radius*2+tol)+tol)
                
            if k%2 == 0:
                pos.append(-domain_y/2+1+j*(radius*2+tol))
            else:
                pos.append(-domain_y/2+1+j*(radius*2+tol)+tol)
                
            pos.append(-domain_z/2 + (k+1) * (radius*2+3*tol))
            pos_list.append(pos)

            vel = []
            vel.append(0)
            vel.append(0)
            vel.append(0)
            vel_list.append(vel)

            fix_list.append(False)


dem1 = DEMSystem(pos_list,vel_list, fix_list)
density = 4000
mass = radius*radius*radius*4/3*math.pi*density
dem1.enforce_init_para(domain_x, domain_y, domain_z, radius , mass)
dem1.set_collision_factor(1e5, 1e8, 1e9)
dem1.set_col_out(False)
#dem1.set_use_ml("ml/my_model.h5")

write_skip = 100
write_count = 0

for i in range(50000):
  ke = dem1.sys_ke()
  print(str(i) + " ke: "+str(ke) )
  dem1.forward(0.0001)
  if i%write_skip == 0:
      dem1.csv_output("fld_4layer_gt/test"+str(write_count))
      #dem1.col_output("col/test"+str(write_count))
     #dem1.print_col_pair("col/test"+str(write_count))
      write_count=write_count+1
      


