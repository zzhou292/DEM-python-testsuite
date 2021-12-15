import csv
import math
def vec3dot(vec1, vec2):
      return vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2]

def vec3minus(vec1,vec2):
      res = []
      res.append(vec1[0]-vec2[0])
      res.append(vec1[1]-vec2[1])
      res.append(vec1[2]-vec2[2])
      return res

def vec3add(vec1,vec2):
      res=[]
      res.append(vec1[0]+vec2[0])
      res.append(vec1[1]+vec2[1])
      res.append(vec1[2]+vec2[2])
      return res

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
            self.restitution_coe = damp


      def collision_handler(self, dir, vel_1, vel_2):
        # stiffness
        penetration = math.sqrt(dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2])
        frc_mag = penetration * self.stiffness
        x_ratio = dir[0] / penetration
        y_ratio = dir[1] / penetration
        z_ratio = dir[2] / penetration
        frc_stiff = []
        frc_stiff.append(frc_mag*(x_ratio))
        frc_stiff.append(frc_mag*(y_ratio))
        frc_stiff.append(frc_mag*(z_ratio))

        # damping
        M  = (self.mass*self.mass)/(self.mass+self.mass);
        K  = self.stiffness;
        C  = 10.*(1./math.sqrt(1. + math.pow(math.pi/math.log(self.restitution_coe), 2)))*math.sqrt(K*M)
        normal = []
        normal.append(dir[0]*x_ratio)
        normal.append(dir[1]*y_ratio)
        normal.append(dir[2]*z_ratio)
        V  = vec3dot(vec3minus(vel_2, vel_1), normal)
        frc_damp = []
        frc_damp.append(C*V*normal[0])
        frc_damp.append(C*V*normal[1])
        frc_damp.append(C*V*normal[2])
        #print("frc_stiff: "+str(frc_stiff))
        #print("frc_damp-: "+str(frc_damp))

        
        return vec3add(frc_stiff,frc_damp)

      def forward(self,timestep):
            n = len(pos_list)

            acc = []
            for i in range(n):
              acc_ele = []
              acc_ele.append(0)
              acc_ele.append(0)
              acc_ele.append(0)
              acc.append(acc_ele)

            # handle collision
            for i in range(n):
                  for j in range(n):
                        if i != j:
                              pos1 = pos_list[i]
                              pos2 = pos_list[j]
                              dir = []
                              dir.append(pos1[0] - pos2[0])
                              dir.append(pos1[1] - pos2[1])
                              dir.append(pos1[2] - pos2[2])
                              if dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2] < (self.radius*2)*(self.radius*2):
                                frc = self.collision_handler(dir, self.vel_list[i],self.vel_list[j])
                                if self.fix_list[i]==False:
                                  acc[i][0] += frc[0] / self.mass
                                  acc[i][1] += frc[1] / self.mass
                                  acc[i][2] += frc[2] / self.mass
                                if self.fix_list[j]==False:
                                  acc[j][0] += - frc[0] / self.mass
                                  acc[j][1] += - frc[1] / self.mass
                                  acc[j][2] += - frc[2] / self.mass

            # handle bounding conditions
            
            
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
            pos.append(0 + k * (radius*2+3*tol))
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
dem1.enforce_init_para(domain_x, domain_y, domain_z, radius , 4)
dem1.set_collision_factor(150, 1e8)

for i in range(2000):
  print(i)
  dem1.forward(0.0025)
  dem1.csv_output("fld/test"+str(i))


