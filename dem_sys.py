import csv
import math
class DEMSystem:
      def __init__(self, pos, vel, fix):
        self.pos_list = pos
        self.vel_list = vel
        self.fix_list = fix

      def enforce_init_para(self, x, y, z, r, m):
        self.domain_x = x
        self.domain_y = y
        self.domain_z = z
        self.radius = r
        self.mass = m

      def set_collision_factor(self, stiff, damp):
            self.stiffness = stiff
            self.damping = damp


      def collision_handler(self, dir_x, dir_y, dir_z):
        penetration = math.sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z)
        frc_mag = penetration * self.stiffness
        x_ratio = dir_x / math.sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z)
        y_ratio = dir_y / math.sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z)
        z_ratio = dir_z / math.sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z)
        frc = []
        frc.append(frc_mag*(x_ratio))
        frc.append(frc_mag*(y_ratio))
        frc.append(frc_mag*(z_ratio))
        return frc

      def forward(self,timestep):
            n = len(pos_list)

            acc = []
            for i in range(n):
              acc_ele = []
              acc_ele.append(0)
              acc_ele.append(0)
              acc_ele.append(0)
              acc.append(acc_ele)

            
            for i in range(n):
                  for j in range(n):
                        if i != j:
                              pos1 = pos_list[i]
                              pos2 = pos_list[j]
                              dir_x = pos1[0] - pos2[0]
                              dir_y = pos1[1] - pos2[1]
                              dir_z = pos1[2] - pos2[2]
                              if dir_x*dir_x+dir_y*dir_y+dir_z*dir_z < (self.radius*2)*(self.radius*2):
                                frc = self.collision_handler(dir_x, dir_y, dir_z)
                                if self.fix_list[i]==False:
                                  acc[i][0] += frc[0] / self.mass
                                  acc[i][1] += frc[1] / self.mass
                                  acc[i][2] += frc[2] / self.mass
                                if self.fix_list[j]==False:
                                  acc[j][0] += - frc[0] / self.mass
                                  acc[j][1] += - frc[1] / self.mass
                                  acc[j][2] += - frc[2] / self.mass
            
            for i in range(n):
              # gravity
              if self.fix_list[i]==False:
                acc[i][2] += -9.8

              self.vel_list[i][0] += acc[i][0] * timestep
              self.vel_list[i][1] += acc[i][1] * timestep
              self.vel_list[i][2] += acc[i][2] * timestep
              self.pos_list[i][0] += self.vel_list[i][0] * timestep
              self.pos_list[i][1] += self.vel_list[i][1] * timestep
              self.pos_list[i][2] += self.vel_list[i][2] * timestep

      def csv_output(self, filename):
          with open(filename+'.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            csvwriter.writerow(['pos_x'+','+'pos_y'+','+'pos_z'+','+'vel_x'+','+'vel_y'+','+'vel_z'])
            n = len(self.pos_list)
            for i in range(n):
                csvwriter.writerow([str(self.pos_list[i][0]) + ',' + str(self.pos_list[i][1]) + ',' + str(self.pos_list[i][2]) + ',' + str(self.vel_list[i][0])+
                ',' + str(self.vel_list[i][1]) + ',' + str(self.vel_list[i][2])  ])



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
dem1.enforce_init_para(domain_x, domain_y, domain_z, radius , 1)
dem1.set_collision_factor(200, 100)

for i in range(100):
  dem1.forward(0.01)
  dem1.csv_output("fld/test"+str(i))


