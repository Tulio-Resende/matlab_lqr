clear all
close all

%%Inicialization
theta_star = [10 75]';  %ground truth

theta_hat = 0*theta_star;   %Theta init
theta_init = theta_hat;
p = 10e6*eye(2,2);    %cov erro matrix init

t = 0:0.01:10;
fi= [sin(t);
    sin(2*t)];
c = (fi'*theta_star)';  


for i = 1:length(t)

    e = c(:,i) - fi(:,i)'*theta_hat;
    theta_hat = theta_hat + (p*fi(:,i)*e)/(1 + fi(:,i)'*p*fi(:,i));

    p = p - (p*fi(:,i)*fi(:,i)'*p)/(1 + fi(:,i)'*p*fi(:,i));

end