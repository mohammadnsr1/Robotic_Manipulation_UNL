clc; 
clear all;
close all;

%%%%%% Import txt file %%%%%%
fileID = fopen('qData.txt');
A = fscanf(fileID,'%f');
A = reshape(A,[7,7835])';  % 7835 x 7 array of joint angles

%%%%%% Define the reference configuration for the serial manipulator %%%%%%
P_Org_tool = [720;640;1910 + 346];
gst_zero = [eye(3) P_Org_tool;0 0 0 1];

%%%%%% Define w,q for each joint in the serial manipulator %%%%%%
% q: is any point on the axis of w (omega);
% w: omega is a unit vector in the direction of the twist axis;

w{1} = [0; 0; 1];
q{1} = [720; 640; 1000 + 346];

w{2} = [0;1;0];
q{2} = q{1};

w{3} = w{1};
q{3} = q{1};

w{4} = w{2};
q{4} = [765; 640; 1550 + 346];

w{5} = w{1};
q{5} = [720; 640; 1850 + 346];

w{6} = w{2};
q{6} = q{5};

w{7} = w{1};
q{7} = q{5};
% 
% % formulate the 6x1 twist vector for each joint
for i = 1:7
    xi{i} = [-cross(w{i},q{i});w{i}];
end

for i = 1:size(A,1)
    theta = A(i,:); % grab angles 
    
    % compute the exponential map for each joint
    % you will need to create the exponential_map function
    for t = 1:7
        gi{t} = exponential_map(xi{t}, theta(t)); 
    end
    
    % Multiply the transformation matrices
    gst = gi{1};
    for t = 2:7
        gst = gst*gi{t};
    end
    gst = gst*gst_zero; 
    position = gst*[0;0;120;1];
    % save the end effector position
    x(i) = position(1);
    y(i) = position(2);
    z(i) = position(3);
end

%%%% Plot the drawing %%%%%%
plot3(x,y,z, '*r')
xlabel('X position');
ylabel('Y position');
zlabel('Z position')

% %%%%%% Write to txt file %%%%%%
fileID = fopen('xyz.txt','w');
fprintf(fileID,'%5f %5f %5f\n',x,y,z);
fclose(fileID);
% Note: relies on exponential_map.m being on the MATLAB path.