# 6DOF-manipulator-kinematic-and-dynamic-simulation-in-Matlab
kinematic and dynamic simulation and analysis of the six-degree-of-freedom robot(6R Robot)

### Forward kinematic:
Denavit-Hartenberg parameters of robot interfaces:

<img width="317" alt="image" src="https://user-images.githubusercontent.com/73820069/155553606-e15659d2-ab85-469c-b151-58a6bc2fb909.png">

### Inverse kinematics(position):
<img width="167" alt="image" src="https://user-images.githubusercontent.com/73820069/155553751-7d399d06-aa5b-4ace-acc4-a9bcb2fa4ce1.png">
<img width="187" alt="image" src="https://user-images.githubusercontent.com/73820069/155553767-30e29fb5-aa11-476e-843e-219b69ca622f.png">

![image](https://user-images.githubusercontent.com/73820069/155553785-0de1738a-0512-4662-8977-770ecbeeeabc.png)

<img width="153" alt="image" src="https://user-images.githubusercontent.com/73820069/155553881-595a336a-a59c-4979-b4a5-e5ad088b54be.png">
<img width="252" alt="image" src="https://user-images.githubusercontent.com/73820069/155553904-04367aa3-49dc-4fb4-9b36-b09de73371fb.png">
<img width="377" alt="image" src="https://user-images.githubusercontent.com/73820069/155553922-73e3ace7-a7aa-46e1-ad39-de34899c814b.png">

Where the D  is equal to:

<img width="302" alt="image" src="https://user-images.githubusercontent.com/73820069/155553956-d0897f99-5eaf-4631-89d6-5dd7225addd5.png">

### Inverse kinematics(Orientation):

<img width="397" alt="image" src="https://user-images.githubusercontent.com/73820069/155554171-4f49963c-2961-457c-9168-c38b17c8bdc2.png">

## Inverse kinematic results for the spatial circle path 

In this project, we have examined inverse kinematics per 100 points for the case where the end effector follows a path corresponding to a space circle. and the rotation matrix is as follows:

R=[1.1 1 1.2;1.3 1.4 1.4;.9 1.2 1]

![image](https://user-images.githubusercontent.com/73820069/155554261-d6d255b1-4d16-4b87-86bc-f01bdcce2a4e.png)

![image](https://user-images.githubusercontent.com/73820069/155554288-9b05bce2-7e27-452b-a8ae-e89b36b0e309.png)

Obtain the movement path of links using interpolation and speed analysis of each link:

2*dx*(x-xc)+2*dy*(y-yc)+2*dz*(z-zc)=0
dx^2+dy^2+dz^2=1
dz=0.2

<img width="264" alt="image" src="https://user-images.githubusercontent.com/73820069/155554377-0143dea8-089b-4b70-ad38-d8e7c89d4c03.png">

<img width="173" alt="image" src="https://user-images.githubusercontent.com/73820069/155554394-11f7dde2-7035-4ae5-87d0-8f8aec62775c.png">

The position-time diagram (q-t) and the velocity-time diagram (dq-t) and the acceleration-time diagram (ddq-t) for each interface shows.


## velocity and Jacobin Kinematics:
For all 6 links:(I=1,2,3,4,5,6):

<img width="230" alt="image" src="https://user-images.githubusercontent.com/73820069/155554552-8fcb6e92-6b77-488d-8109-1657b2b6c66f.png">



#### Robot Dynamics:
To determine the Euler-Lagrange equations, it is necessary to form the Lagrangian system, which is the difference between kinetic energy and potential energy. 

##### Kinetic energy:

<img width="241" alt="image" src="https://user-images.githubusercontent.com/73820069/155554603-b8125748-0b96-4392-a592-d6f3e8099035.png">

Where m is the mass of the whole body and v, w are angular velocity vectors and linear velocities, respectively. The symmetric matrix I is also a 3 * 3 matrix called the inertial tensor. Using the rotation matrices for each link so that we have kinetic energy for:

<img width="411" alt="image" src="https://user-images.githubusercontent.com/73820069/155554648-3fbe5495-054b-459f-9547-e2619df5f02e.png">

D is called the inertia matrix, which is a symmetric and definite positive matrix.

##### potential energy:
To calculate the potential energy under rigid dynamic conditions, the only source of energy is the gravitational potential. For the i link, the following is:

<img width="186" alt="image" src="https://user-images.githubusercontent.com/73820069/155554728-4ef3e56b-d62a-46bb-899a-925f175ef5c8.png">

Which g is a vector that shows the direction of gravity in the inertia coordinate system
and the vector r determines the coordinates of the center of mass of the link.


### Motion equations:
To write the Euler-Lagrange equations in a particular case, two conditions must first be met.
Kinetic energy is a quadratic function of the diff (q) vector as follows:

<img width="286" alt="image" src="https://user-images.githubusercontent.com/73820069/155554808-39de0d2f-6368-4837-aca7-1581f9b72e05.png">

Which dij is matrix components D.
2. The potential energy is independent of diff (q)
Therefore, Lagrange for the robot is obtained as follows:

<img width="262" alt="image" src="https://user-images.githubusercontent.com/73820069/155554864-7460c635-c53a-41f1-8e80-4de1500ce1e3.png">

The Euler-Lagrange equation is obtained as follows, in which c is called the Christopher symbol (type one), which is a partial derivation of the matrix of inertia D and ø is the gravity vector obtained by deriving the potential energy :

<img width="382" alt="image" src="https://user-images.githubusercontent.com/73820069/155554924-7bb6e987-d273-40ad-9e8b-61009da68938.png">

As a result, the Euler-Lagrange equations can be written as follows:

<img width="432" alt="image" src="https://user-images.githubusercontent.com/73820069/155554953-70fec762-99e1-4873-91f2-add2241d95ff.png">

### Robot simulation and animation:
To simulate the robot, we need the coordinates and position of the joints at any given moment, which we obtain from the direct kinematic solution and the fourth column of the transformation matrices. The path of the space circle is achieved by the final performer. The film is attached.

![image](https://user-images.githubusercontent.com/73820069/155555021-c4c3238b-14e2-4668-95e7-bd2a7445c3f0.png)

## How to run the program:
Please run the following programs:

    1-one_forward_kinematic_and_jacobian
    2-tow_invert_kinematics
    3-three_invert_dynamic
    4-four_animate

