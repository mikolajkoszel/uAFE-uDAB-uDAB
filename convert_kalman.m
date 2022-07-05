function [amplitude, frequency, phase, init_val] = convert_kalman(vector, accuracy)
omega = 2*pi*50;

vector = vector(1:end-1,:);
T = [];
for k = 1:numel(vector)/2
T_diag = [cos(omega*(1/3)*k) -sin(omega*(1/3)*k);
          sin(omega*(1/3)*k)  cos(omega*(1/3)*k)];
T = blkdiag(T,T_diag);
end

frequency = [];
for k = 1:numel(vector)/2
    amplitude(k) = sqrt(vector(k*2-1).*vector(k*2-1) + vector(k*2).*vector(k*2));
    if(amplitude(k) > accuracy)
		  frequency = [frequency k];
	 end
end
amplitude(amplitude <= accuracy) = [];
init_val = [0 0 0];
for j = 1:3
	 for k = 1:numel(frequency)
        phase(k,j) = atan(vector(frequency(k)*2-1) / vector(frequency(k)*2));
        if(vector(frequency(k)*2) < 0) 
            phase(k,j) = phase(k,j) + pi;
        end
        init_val(j) = init_val(j) + vector(frequency(k)*2-1);
    end
    vector = T*vector;
end

frequency = -omega * frequency;
end