close all; clear; %clear classes; %clc;
robot = importrobot("../robot.urdf");
robot = robot(1);
%show(robot);
%showdetails(robot);

%Create user interfase object for SRD
SRD = SRDuserinterface;
%Create ground link
Ground = SRD.CreateGroundLink();

bodies = [Ground];

%Creating links
body_count = robot.NumBodies;
body_structs = [];
for body_idx=1:body_count
    body = robot.Bodies(body_idx);
    body = body{1};
    
    RelativeCoM = reshape(body.CenterOfMass,[3,1]); 
    Mass = body.Mass;
    Name = body.Name;

    Inertia = eye(3);
    
    inertia_vec = body.Inertia;
    
    Inertia(1,1) = inertia_vec(1);
    Inertia(2,2) = inertia_vec(2);
    Inertia(3,3) = inertia_vec(3);
    Inertia(3,2) = inertia_vec(4);
    Inertia(3,1) = inertia_vec(5);
    Inertia(2,1) = inertia_vec(6);
    
        
    RelativeBase = [0; 0; 0];
    RelativeFollower = [0; 0; 1];
    
    body_struct = struct('RelativeBase',RelativeBase, 'RelativeFollower',RelativeFollower,...
        'RelativeCoM', RelativeCoM, 'Mass',Mass, 'Inertia',Inertia, 'Name',Name);
    body_structs = [body_structs;body_struct];
end

%Parse joint info
for body_idx=1:body_count
    body = robot.Bodies(body_idx);
    body = body{1};
    
    parent_link = body.Parent;
    
    body_struct = body_structs(body_idx);
    if length(parent_link)~=0
        joint_info = body.Joint;
        
        joint_Type = joint_info.Type;
        joint_axis = joint_info.JointAxis;
        parent_name = parent_link.Name;
        
        name_matcher = @(i) strcmp(bodies(i).Name,parent_name);

        match = arrayfun(name_matcher, 1:numel(bodies));
        parent_index = find(match);
        
        
        parent_obj = bodies(parent_index);
        
        joint_name = "";
        
        if strcmp(joint_Type,'floating')   
                joint_name = 'FloatingBase_6dof';
        else
            
            if strcmp(joint_Type,'planar')
                joint_name = 'planar';
            end
            
            if strcmp(joint_Type,'pivot') || strcmp(joint_Type,'revolute')
                joint_name = 'pivot';
            end
            
            if strcmp(joint_Type,'prismatic')
                joint_name = 'prismatic';
            end
            
            
            if joint_info.JointAxis(1)==1 && joint_info.JointAxis(2)==1 && joint_info.JointAxis(3)==1
                joint_name = [joint_name,'_XYZ'];
            else
                if joint_info.JointAxis(1)==1
                    joint_name = [joint_name,'X'];
                end
                if joint_info.JointAxis(2)==1
                    joint_name = [joint_name,'Y'];
                end
                if joint_info.JointAxis(3)==1
                    joint_name = [joint_name,'Z'];
                end
            end
            
        end
        
        gen_coords = -1;
        switch joint_name
                    case 'none'
                gen_coords = 0;
            case {'FloatingBase_6dof', 'FloatingBase_6dof_ZYX'}
                gen_coords = 6;
            case {'abs_spherical', 'prismatic_XYZ', ...
                  'planarX', 'planarY', 'planarZ', ...
                  'abs_planarX', 'abs_planarY', 'abs_planarZ'}
                gen_coords = 3;
            case {'pivotXY', 'pivotYZ', 'pivotZX'}
                gen_coords = 2;
            case {'pivotX', 'pivotY', 'pivotZ', 'abs_pivotX', 'abs_pivotY', 'abs_pivotZ', ...
                    'prismaticX', 'prismaticY', 'prismaticZ'}
                gen_coords = 1;
            otherwise
                warning('Invalid joint type');
        end    
           
        body_obj = SRDLinkWithJoint('JointType',joint_name,'Order', body_idx,'FileName', [],...
        'LinkParametersStructure', body_struct, 'ParentLink', parent_obj,'ParentFollowerNumber', 1);
        
        body_obj.SetUsedGenCoordinates(gen_coords);

        bodies = [bodies; body_obj];
    
    end
end

LinkArray = bodies;

%InitialPosition = [pi/4; -2*pi/3; 1*pi/5];
%InitialPosition = [pi/4; -2*pi/3; 1*pi/5]; %Define initial position of the robot
InitialPosition = [pi/6; -pi/6; pi/6]; %Define initial position of the robot
AxisLimits = [-1; 1; -1; 1; 0; 2]; %Set axis limits for the snapshot
ViewAngle = [0, 45];

SRD.CreateRobotStructure(LinkArray, InitialPosition, AxisLimits, ViewAngle); %Create the robot
axis equal;


