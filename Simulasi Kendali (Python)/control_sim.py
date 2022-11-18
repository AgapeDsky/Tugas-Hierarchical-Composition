import matplotlib.pyplot as plt

#controller
kp = 2
ki = 1
kd = 0
sampling_time = 0.008

#plant
tau = 1.2; T = 0.008;

##############

target = 10.;
err = 0.; err_1 = 0.; err_2 = 0.;
effort = 0.; effort_1 = 0.; effort_2 = 0.;
output = 0.; output_1 = 0.;

a = (4*kd+2*sampling_time*kp+sampling_time*sampling_time*ki)/(2*sampling_time);
b = (2*sampling_time*sampling_time*ki-8*kd)/(2*sampling_time);
c = (4*kd-2*sampling_time*kp+sampling_time*sampling_time*ki)/(2*sampling_time);
d = 1.;

print(a,b,c,d)

def compute_control():
    return a*err+b*err_1+c*err_2+d*effort_2;

def compute_plant():
    return effort_1/(2*tau+T)*T + effort/(2*tau+T)*T - output*(T-2*tau)/(2*tau+T);

##############

fig = plt.figure()
i = 0
x = list()
y = list()

while i < 2000 :
    err = target-output;
    effort = compute_control();
    output = compute_plant();
    i = i + 1
    y.append(output)
    x.append(i*T)

    err_2 = err_1;
    err_1 = err;
    effort_2 = effort_1;
    effort_1 = effort;
    output_1 = output;

plt.plot(x,y)
plt.title('Velocity Control')
plt.xlabel('Time Stamp (s)')
plt.ylabel('Velocity')
plt.show() 
    

