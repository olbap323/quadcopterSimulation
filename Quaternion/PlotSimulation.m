

% state-vector: X = [u v w p q r xf yf zf e0 ex ey ez w1 w2 w3 w4]'
u = X(1,:);
v = X(2,:);
w = X(3,:);
p = X(4,:);
q = X(5,:);
r = X(6,:);
x_f = X(7,:);
y_f = X(8,:);
z_f = X(9,:);
Q = X(10:13,:);
w1 = X(14,:);
w2 = X(15,:);
w3 = X(16,:);
w4 = X(17,:);

eAngs = zeros(3,N);

for i = 1 : N,
   eAngs(:,i) = Q2eAng( Q(:,i) ); 
end
phi = eAngs(1,:);
theta = eAngs(2,:);
psi = eAngs(3,:);

[~,N_h] = size(ei_p);
t_h = linspace(t0,tf,N_h);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% veolicities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Position
figure(fN);
clf;
subplot(3,1,1);
plot(t,x_f);
if gridsOn,
   grid on; 
end
ylabel('$x_{f}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,2);
plot(t,y_f);
if gridsOn,
   grid on; 
end
ylabel('$y_{f}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,3);
plot(t,z_f);
if gridsOn,
   grid on; 
end
ylabel('$z_{f}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)

%%%%%% orientation (euler angles)
fN = fN + 1;
figure(fN);
clf;
subplot(3,1,1);
plot(t,phi*r2d,'b');
hold on;
plot(tp,u_att(1,:)*r2d,':r');
hold off;
if gridsOn,
   grid on; 
end
ylabel('$\phi$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,2);
plot(t,theta*r2d,'b');
hold on;
plot(tp,u_att(2,:)*r2d,':r');
hold off;
if gridsOn,
   grid on; 
end
ylabel('$\theta$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,3);
plot(t,psi*r2d,'b');
hold on;
plot(tp,u_att(3,:)*r2d,':r');
hold off;
if gridsOn,
   grid on; 
end
ylabel('$\psi$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)

%%%%%% euler angles error
fN = fN + 1;
figure(fN);
clf;
subplot(3,1,1);
plot(tp,(u_att(1,:) - phi(1:fs_att/fs_pos:end-1))*r2d,'b');
if gridsOn,
   grid on; 
end
ylabel('$e_{\phi}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,2);
plot(tp,(u_att(2,:) - theta(1:fs_att/fs_pos:end-1))*r2d,'b');
if gridsOn,
   grid on; 
end
ylabel('$e_{\theta}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,3);
plot(tp,(u_att(3,:) - psi(1:fs_att/fs_pos:end-1))*r2d,'b');
if gridsOn,
   grid on; 
end
ylabel('$e_{\psi}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% veolicities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% linear velocity
fN = fN + 1;
figure(fN);
clf;
subplot(3,1,1);
plot(t,u);
ylabel('$u$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,2);
plot(t,v);
ylabel('$v$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,3);
plot(t,w);
ylabel('$w$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
%%%%%% angular velocity
fN = fN + 1;
figure(fN);
clf;
subplot(3,1,1);
plot(t,p);
ylabel('$p$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,2);
plot(t,q);
ylabel('$q$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,3);
plot(t,r);
ylabel('$r$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% motor angular rate %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fN = fN + 1;
figure(fN);
clf;
subplot(2,2,1);
plot(t,w1*rads2rpm);
ylabel('$w_{1}~~rpm$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(2,2,2);
plot(t,w2*rads2rpm);
ylabel('$w_{2}~~rpm$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(2,2,3);
plot(t,w3*rads2rpm);
ylabel('$w_{3}~~rpm$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(2,2,4);
plot(t,w4*rads2rpm);
ylabel('$w_{4}~~rpm$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Non linear attitude controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% lyapunov
%omega (lyapunov redesign sign parameter)
fN = fN + 1;
figure(fN);
clf;
for i = 1:3,
    subplot(3,1,i);
    plot(t,sign(omega_lyap(i,:)));
    ylab = strcat('$ \omega_{L,',num2str(i),'}$');
    ylabel(ylab,'interpreter','latex','fontsize',labelFont);
end
%delta (uncertainty)
fN = fN + 1;
figure(fN);
clf;
for i = 1:3,
    subplot(3,1,i);
    plot(t,delta(i,:));
    ylab = strcat('$ \delta_{',num2str(i),'}$');
    ylabel(ylab,'interpreter','latex','fontsize',labelFont);
end

fN = fN + 1;
figure(fN);
clf;
for i = 1:4,
    subplot(2,2,i);
    plot(t,u_plant(i,:)*rads2rpm);
    ylab = strcat('$ \omega_{d,',num2str(i),'}$');
    ylabel(ylab,'interpreter','latex','fontsize',labelFont);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% integral errors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% position
fN = fN + 1;
figure(fN);
clf;
subplot(3,1,1);
plot(t_h,ei_p(1,:));
ylabel('$e_{i,1}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,2);
plot(t_h,ei_p(2,:));
ylabel('$e_{i,2}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,3);
plot(t_h,ei_p(3,:));
ylabel('$e_{i,3}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)

%%%%%% attitude
fN = fN + 1;
figure(fN);
clf;
subplot(3,1,1);
plot(t,ei_a(1,:));
ylabel('$e_{ia,1}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,2);
plot(t,ei_a(2,:));
ylabel('$e_{ia,2}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)
subplot(3,1,3);
plot(t,ei_a(3,:));
ylabel('$e_{ia,3}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% state estimator (EKF) xhat %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uhat = Xhat(1,:);
vhat = Xhat(2,:);
what = Xhat(3,:);
phat = Xhat(4,:);
qhat = Xhat(5,:);
rhat = Xhat(6,:);
x_fhat = Xhat(7,:);
y_fhat = Xhat(8,:);
z_fhat = Xhat(9,:);
Qhat = Xhat(10:13,:);
w1hat = Xhat(14,:);
w2hat = Xhat(15,:);
w3hat = Xhat(16,:);
w4hat = Xhat(17,:);

No = length(w4hat);
t_hat = linspace(t0,tf,No);


eAngsHat = zeros(3,No);
Ptrace = zeros(1,No);

for i = 1 : No,
   eAngsHat(:,i) = Q2eAng( Qhat(:,i) );
   Ptrace(i) = trace(P(:,:,i));
end
phihat = eAngsHat(1,:);
thetahat = eAngsHat(2,:);
psihat = eAngsHat(3,:);


fN = fN + 1;
figure(fN);
clf;

subplot(3,1,1);
plot(t_hat,x_fhat);
if gridsOn,
   grid on; 
end
ylabel('$\hat{x_{f}}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)

subplot(3,1,2);
plot(t_hat,y_fhat);
if gridsOn,
   grid on; 
end
ylabel('$\hat{y_{f}}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)

subplot(3,1,3);
plot(t_hat,z_fhat);
if gridsOn,
   grid on; 
end
ylabel('$\hat{z_{f}}$','interpreter','latex','fontsize',labelFont)
xlabel('t','interpreter','latex','fontsize',labelFont)


Pt  = zeros(1,No);

for i = 1:No,
    Pt(i) = trace(P(:,:,i));
end
fN = fN + 1;
figure(fN);
clf;
plot(t_hat,Pt);