"""controller_bayes_braitenberg controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import csv
import random
from collections import defaultdict
import statistics


TIMESTEP = 64 

def run_robot(robot):
 

#####Enable motors
 
    right_motor = robot.getDevice('motor_1') 
    left_motor = robot.getDevice('motor_2')
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
#####Enable distance sensors 
    
    distance_sensors = []
    distance_sensors_names = ['ds_right_1', 'ds_right_2', 'ds_right_3',
                              'ds_left_1' , 'ds_left_2',  'ds_left_3']
    
    for ind in range(6):
        distance_sensors.append(robot.getDevice(distance_sensors_names[ind]))
        distance_sensors[ind].enable(TIMESTEP)
        
#####Funcion Re Mapeo


    def calcular_dir(left_distance_val, right_distance_val):
        if left_distance_val[2] > 0:
            return -3
        elif left_distance_val[1] > 0:
            return -2
        elif left_distance_val[0] > 0:
            return -1
        elif right_distance_val[0] > 0:
            return 1
        elif right_distance_val[1] > 0:
            return 2
        elif right_distance_val[2] > 0:
            return 3
        else:
            return 0  # Ningún sensor detectó nada
             
       
#####Funcion Vrot

    # Al inicio: cargar y procesar CSV
    modelo = defaultdict(list)
    
    with open("vrot_data_unique.csv", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            dir_val = int(row["Dir"])
            prox_val = int(row["Prox"])
            vrot_val = float(row["Vrot"])
            modelo[(dir_val, prox_val)].append(vrot_val)
            
    # Luego: calcular tabla con media y sigma
    tabla = {}
    for clave, valores in modelo.items():
        mu = statistics.mean(valores)
        sigma = statistics.stdev(valores) if len(valores) > 1 else 0.5  # evitar sigma=0
        tabla[clave] = (mu, sigma)  
        
        
        
####### Main loop:
   
    
    while robot.step(TIMESTEP) != -1:
        

        """
        PROX
        
        Se recogen los valores de los sensores, 'right_distance_val' y
        'left_distance_val' que pertenecen al rango [0,10] de los Z.
        
        0 a 1,000 y /100 para que sea 10. Despues aplico la formula del paper
        
        esta en el intervalor [0,1,2,3,4,5,6,7,8,9] y no llega a diez por
        la funcion floor
        """
        
        right_distance_val = []
        left_distance_val = []
        right_distance = 0
        left_distance = 0 
        
        for sensoidrc in range(3):
            right_distance_val.append(
            math.floor(10 - (distance_sensors[sensoidrc].getValue()/100)))
            
        for sensoirzq in range(3):
            left_distance_val.append(
            math.floor(10 - (distance_sensors[sensoirzq + 3].getValue()/100)))
            

        max_prox = max(right_distance_val[0], right_distance_val[1], right_distance_val[2],
        left_distance_val[0], left_distance_val[1], left_distance_val[2])
        
        prox_val = max_prox
            
            
        """
        DIR 
        
        Voy a asociar a los sensores un peso
        
          NOMBRE   // GRADO // PESO  
        
        ds_left_3  // -90°   // -3
        ds_left_2  // -45°   // -2
        ds_left_1  // -10°   // -1
        ds_right_1 //  10°   //  1
        ds_right_2 //  45°   //  2
        ds_right_3 //  90°   //  3
        
        esta en el intervalo [-3, -2, -1, 0, 1, 2, 3] 
        
        """


        dir_val = calcular_dir(left_distance_val, right_distance_val)
  
  
  
        """
        VROT
        
        Se calcula a partir de la tabla de medias y desviaciones
        calculadas para cada valor de dir y prox 
        
        """  
        if (dir_val, prox_val) in tabla:
            media, sigma = tabla[(dir_val, prox_val)]
            vrot = random.gauss(media, sigma)
        else:
            vrot = 0 
            
        vtrans = 3
        
        Mr = math.floor(vtrans + vrot)
        Ml = math.floor(vtrans - vrot) 

        
        right_motor.setVelocity(Mr)
        left_motor.setVelocity(Ml)
                       
        
########Impresiones         
 
        print("===============================")
        
        for dist in range(3):
            print(f"right_distance_val {dist + 1}: {right_distance_val[dist]}")
            
        for dist in range(3):
            print(f"left_distance_val {dist + 1}: {left_distance_val[dist]}")
            
            
        print("right_speed: {}".format(Mr))
        print("left_speed: {}".format(Ml))
        print("dir_val: {}".format(dir_val))
        print("prox_val: {}".format(prox_val))       
        print("vrot: {}".format(vrot))
        print(tabla)


if __name__ == "__main__":


    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
    