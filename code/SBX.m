D_multitask = 0;
mu = 0;
count = 0;
u = rand(1,D_multitask);

cf = zeros(1,D_multitask);

cf(u<=0.5)=(2*u(u<=0.5)).^(1/(mu+1));

cf(u>0.5)=(2*(1-u(u>0.5))).^(-1/(mu+1));

child(count) = crossover(child(count),population(p1),population(p2),cf);

child(count+1) = crossover(child(count+1),population(p2),population(p1),cf);

function object=crossover(object,p1,p2,cf)

object.rnvec=0.5*((1+cf).*p1.rnvec + (1-cf).*p2.rnvec);

% ½Ø¶Ï·¶Î§

object.rnvec(object.rnvec>1)=1;

object.rnvec(object.rnvec<0)=0;

end

