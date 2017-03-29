R1 = [0.972515 0.232583 0.010963;-0.232777 0.972276 0.0222336;-0.00548789 -0.0241744 0.999693];
x1 = [-0.324846 -0.292416 9.03128]';
n1t = [-0.223367 0.974634 0.0139989];

R2 = [0.976689 0.200762 0.0759801;0.214583 -0.903709 -0.370492;-0.00571676 0.378159 -0.925723];
x2 = [-0.301028 1.68173 8.87898]';
n2t = [-0.227172 0.946039 0.23109];

h1 = R1 + x1*n1t;
h2 = R2 + x2*n2t;

h = [1.89416 -0.152288 0.0116279;-0.303517 1.24567 0.0328782;-3.66621 15.9099 2.04105];

