import csv
import math
import os
import tensorflow as tf
import numpy as np
import keras
from keras import backend as K

def maximum(a, b):
     
    if a >= b:
        return [a,0]
    else:
        return [b,1]

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
        self.col_out = False
        self.use_ml = False
        self.enforce_z_up_bound = False
      
      def set_z_up_bound(self, enforce):
            self.enforce_z_up_bound = enforce

      def enforce_init_para(self, x, y, z, r, m):
        self.domain_x = x
        self.domain_y = y
        self.domain_z = z
        self.radius = r
        self.mass = m

      def set_collision_factor(self, stiff, damp, w_stiff):
            self.stiffness = stiff
            self.damp = damp
            self.w_stiff = w_stiff
            #print("stiff:"+str(stiff))
            #print("damp:"+str(damp))
            #print("w_stiff:"+str(w_stiff))
        
      def set_col_out(self, write):
          self.col_out = write

      def set_use_ml(self, ml_filename):
          self.ml_path = ml_filename
          self.use_ml = True
          self.ml_model = keras.models.load_model(self.ml_path)

      def collision_handler(self, dir, vel_1, vel_2):
        # stiffness
        dist = math.sqrt(dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2])
        penetration = 2*self.radius - dist
        #print("penetration: "+str(penetration))
        frc_mag = penetration * self.stiffness
        x_ratio = dir[0] / dist
        y_ratio = dir[1] / dist
        z_ratio = dir[2] / dist
        frc_stiff = []
        frc_stiff.append(frc_mag*(x_ratio))
        frc_stiff.append(frc_mag*(y_ratio))
        frc_stiff.append(frc_mag*(z_ratio))

        # damping
        M  = (self.mass*self.mass)/(self.mass+self.mass)
        K  = self.stiffness
        C  = 2.*(1./math.sqrt(1. + math.pow(math.pi/math.log(self.damp), 2)))*math.sqrt(K*M)
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


      def collision_handler_ml(self, pred_list):
        input_np = np.array(pred_list)
        input_tensor = tf.convert_to_tensor(input_np, dtype=tf.float32) 
        #print(input_tensor)

        frc_tensor = self.ml_model.predict(input_tensor)
        frc_list = frc_tensor.tolist()
        return frc_list



      def forward(self,timestep):
            n = len(self.pos_list)

            acc = []
            
            
            self.dir_store = []
            self.vel1_store = []
            self.vel2_store = []
            self.frc_store = []
                
            
            
            for i in range(n):
              acc_ele = []
              acc_ele.append(0)
              acc_ele.append(0)
              acc_ele.append(0)
              acc.append(acc_ele)

            # handle collision
            if self.use_ml == False:
              for i in range(n):
                  for j in range(n):

                          if i != j:
                                pos1 = self.pos_list[i]
                                pos2 = self.pos_list[j]
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
                                    
                                  if self.col_out == True:
                                      self.dir_store.append(dir)
                                      self.vel1_store.append(self.vel_list[i])
                                      self.vel2_store.append(self.vel_list[j])
                                      self.frc_store.append(frc)

            else:
              pred_list = []
              i_list = []
              j_list = []
              for i in range(n):
                for j in range(n):
                          
                  if i != j:
                    pos1 = self.pos_list[i]
                    pos2 = self.pos_list[j]
                    dir = []
                    dir.append(pos1[0] - pos2[0])
                    dir.append(pos1[1] - pos2[1])
                    dir.append(pos1[2] - pos2[2])
                    if dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2] < (self.radius*2)*(self.radius*2):
                      pred_ele = []
                      pred_ele.append(dir[0])
                      pred_ele.append(dir[1])
                      pred_ele.append(dir[2])
                      pred_ele.append(self.vel_list[i][0])
                      pred_ele.append(self.vel_list[i][1])
                      pred_ele.append(self.vel_list[i][2])
                      pred_ele.append(self.vel_list[j][2])
                      pred_ele.append(self.vel_list[j][2])
                      pred_ele.append(self.vel_list[j][2])
                      pred_list.append(pred_ele)
                      i_list.append(i)
                      j_list.append(j)

              if len(pred_list)!=0:
                frc = self.collision_handler_ml(pred_list)
                for i in range(len(pred_list)):
                  i_temp = i_list[i]
                  j_temp = j_list[i]
                  frc_temp = frc[i]

                  if self.fix_list[i_temp]==False:
                    acc[i_temp][0] += frc_temp[0] / self.mass
                    acc[i_temp][1] += frc_temp[1] / self.mass
                    acc[i_temp][2] += frc_temp[2] / self.mass
                  if self.fix_list[j_temp]==False:
                    acc[j_temp][0] += - frc_temp[0] / self.mass
                    acc[j_temp][1] += - frc_temp[1] / self.mass
                    acc[j_temp][2] += - frc_temp[2] / self.mass
                          

                                
                
                                    
            # handle bounding conditions
            for i in range(n):
              if self.fix_list[i] == False:
                  pos = self.pos_list[i]
                  x_hit_bd = False
                  y_hit_bd = False
                  z_hit_bd = False
                  
                  x_pene = 0
                  y_pene = 0
                  z_pene = 0
 
                  
                  x_check = maximum(abs(pos[0]+self.radius),abs(pos[0]-self.radius))
                  y_check = maximum(abs(pos[1]+self.radius),abs(pos[1]-self.radius))
                  z_check = maximum(abs(pos[2]+self.radius),abs(pos[2]-self.radius))
                  
                  if x_check[0]>=self.domain_x/2:
                    x_hit_bd = True
                  elif y_check[0]>=self.domain_y/2:
                    y_hit_bd = True
                  elif z_check[0]>=self.domain_z/2:
                    z_hit_bd = True

            
                  if x_hit_bd == True:
                    if x_check[1]==0:
                        x_frc = (abs(pos[0]+self.radius) - self.domain_x/2)*self.w_stiff
                        acc[i][0] = acc[i][0]-x_frc/self.mass
                    elif x_check[1]==1:
                        x_frc = (abs(pos[0]-self.radius) - self.domain_x/2)*self.w_stiff
                        acc[i][0] = acc[i][0]+x_frc/self.mass 
                        
                        
                        
                  if y_hit_bd == True:
                    if y_check[1]==0:
                        y_frc = (abs(pos[1]+self.radius) - self.domain_y/2)*self.w_stiff
                        acc[i][1] = acc[i][1]-y_frc/self.mass
                    elif y_check[1]==1:
                        y_frc = (abs(pos[1]-self.radius) - self.domain_y/2)*self.w_stiff
                        acc[i][1] = acc[i][1]+y_frc/self.mass  
                    
                    
                  if z_hit_bd == True:
                    if self.enforce_z_up_bound == True:
                      if z_check[1]==0:
                          z_frc = (abs(pos[2]+self.radius) - self.domain_z/2)*self.w_stiff
                          acc[i][2] = acc[i][2]-z_frc/self.mass
                    if z_check[1]==1:
                        z_frc = (abs(pos[2]-self.radius) - self.domain_z/2)*self.w_stiff
                        acc[i][2] = acc[i][2]+z_frc/self.mass  
            
            
            # gravity and integration
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
                
      def col_output(self,filename):
          with open(filename+'.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            csvwriter.writerow(['dir_x'+','+'dir_y'+','+'dir_z'+','+'vel1_x'+','+'vel1_y'+','+'vel1_z'+','+'vel2_x'+','+
                                'vel2_y'+','+'vel2_z'+','+'frc_x'+','+'frc_y'+','+'frc_z'])
            n = len(self.dir_store)
            for i in range(n):
                csvwriter.writerow([str(self.dir_store[i][0]) + ',' + str(self.dir_store[i][1]) + ',' + str(self.dir_store[i][2]) + ',' 
                                    + str(self.vel1_store[i][0])+',' + str(self.vel1_store[i][1]) + ',' + str(self.vel1_store[i][2]) + ','
                                    + str(self.vel2_store[i][0]) + ','+ str(self.vel2_store[i][1])+','+str(self.vel2_store[i][2]) + ','
                                    + str(self.frc_store[i][0]) + ','+ str(self.frc_store[i][1])+','+str(self.frc_store[i][2])])
                
      def sys_ke(self):
          n = len(self.pos_list)
          sum = 0
          for i in range(n):
              sum = sum + 0.5 * self.mass * (self.vel_list[i][0] * self.vel_list[i][0] + 
                                             self.vel_list[i][1] * self.vel_list[i][1] +
                                             self.vel_list[i][2] * self.vel_list[i][2])
          return sum