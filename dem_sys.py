import csv
class DEMSystem:
      def __init__(self, pos, vel, fix):
        self.pos_list = pos
        self.vel_list = vel
        self.fix_list = fix

      def enforce_init_para(self, x, y, z, r):
        self.domain_x = x
        self.domain_y = y
        self.domain_z = z
        self.radius = r

      def set_collision_factor(self, stiff, damp):
            self.stiffness = stiff
            self.damping = damp

      def forward(self,timestep):
            n = len(pos_list)
            for i in range(n):
                  for j in range(n):
                        if i != j:
                              pos1 = pos_list[i]
                              pos2 = pos_list[j]
                              
                              

    
      

      def csv_output(self, filename):
          with open(filename+'.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            csvwriter.writerow(['pos_x'+','+'pos_y'+','+'pos_z'+','+'vel_x'+','+'vel_y'+','+'vel_z'])
            n = len(self.pos_list)
            for i in range(n):
                csvwriter.writerow([str(pos_list[i][0]) + ',' + str(pos_list[i][1]) + ',' + str(pos_list[i][2]) + ',' + str(vel_list[i][0])+
                ',' + str(vel_list[i][1]) + ',' + str(vel_list[i][2])  ])



domain_x = 6
domain_y = 6
domain_z = 6
radius = 0.2
tol = 0.05
pos_list = []
vel_list = []
fix_list = []

num = int(domain_x / (2 * radius + tol))
for k in range (0,2,1):
    for j in range(0, num, 1):
        for i in range(0, num, 1):
            pos = []
            pos.append(-2+i*(radius*2+tol))
            pos.append(-2+j*(radius*2+tol))
            pos.append(0 + k * (radius*2+tol))
            pos_list.append(pos)

            vel = []
            vel.append(0)
            vel.append(0)
            vel.append(0)
            vel_list.append(vel)

            if k == 0:
              fix_list.append(True)
            else:
              fix_list.append(False)

dem1 = DEMSystem(pos_list,vel_list, fix_list)
dem1.set_collision_factor(5000, 1000)


dem1.csv_output("test")
