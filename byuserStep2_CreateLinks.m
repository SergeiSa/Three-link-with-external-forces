RelativeBase = [0; 0; 0];        
RelativeFollower = [1; 0; 0];   
RelativeCoM = [0.5; 0; 0]; 
Mass = 1;
Inertia = eye(3);
Name = 'Shin';
save('datafile_Shin', 'RelativeBase', 'RelativeFollower', 'RelativeCoM', 'Mass', 'Inertia', 'Name');

RelativeBase = [0; 0; 0];        
RelativeFollower = [1; 0; 0];   
RelativeCoM = [0.5; 0; 0];   
Mass = 1;
Inertia = eye(3);
Name = 'Hip';
save('datafile_Hip', 'RelativeBase', 'RelativeFollower', 'RelativeCoM', 'Mass', 'Inertia', 'Name');

RelativeBase = [0; 0; 0];        
RelativeFollower = [1; 0; 0];   
RelativeCoM = [0.5; 0; 0]; 
Mass = 1;
Inertia = eye(3);
Name = 'Torso';
save('datafile_Torso', 'RelativeBase', 'RelativeFollower', 'RelativeCoM', 'Mass', 'Inertia', 'Name');


