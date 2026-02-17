function [ESBI, EVBI, RICI] = init_ins(mINS, frax)
    % Initialize output
    ESBI = zeros(3,1);
    EVBI = zeros(3,1);
    RICI = zeros(3,1);

    if mINS == 0
        return;
    end

    % Predefined 9x9 initial covariance matrix (same as in C++)
    PP0 = [
          20.701,     0.12317,    0.10541,...
		  6.3213E-02, 2.2055E-03, 1.7234E-03,...
		  1.0633E-03, 3.4941E-02,-3.5179E-02;

          0.12317,    20.696,    -0.27174,...
		  4.8366E-03, 5.9463E-02,-1.3367E-03,...
	     -3.4903E-02, 2.6112E-03,-4.2663E-02;
        
          0.10541,   -0.27174,    114.12,...
		  5.6373E-04,-8.3147E-03, 5.4059E-02,...
		  1.5496E-02, 7.6463E-02,-3.5302E-03;

          6.3213E-02, 4.8366E-03, 5.6373E-04,...
		  1.9106E-03, 8.0945E-05, 1.9810E-06,...
		  2.5755E-04, 2.8346E-03,-5.6482E-04;

          2.2055E-03, 5.9463E-02,-8.3147E-03,...
		  8.0945E-05, 1.7201E-03,-1.5760E-05,...
		 -2.8341E-03, 2.6478E-04,-1.0781E-03;

          1.7234E-03,-1.3367E-03, 5.4059E-02,...
		  1.9810E-06,-1.5760E-05, 3.0070E-03,...
		  4.1963E-04,-1.3297E-04, 4.1190E-05;
 
		  1.0638E-03,-3.4903E-02, 1.5496E-02,...
		  2.5755E-04,-2.8341E-03, 4.1963E-04,...
		  5.4490E-02,-1.8695E-03, 8.9868E-04;
 
		  3.4941E-02, 2.6112E-03, 7.6463E-02,...
		  2.8346E-03, 2.6478E-04,-1.3297E-04,...
		 -1.8695E-03, 5.2819E-02, 1.0990E-02;
 
		 -3.5179E-02,-4.2663E-02,-3.5302E-03,...
		 -5.6482E-04,-1.0781E-03, 4.1190E-05,...
		  8.9868E-04, 1.0990E-02, 0.1291;
    ];
    
    % Cholesky-like lower-triangular square root of PP0
    APP0 = zeros(9,9);
        for i = 1:9
            for j = 1:9
                if j < i
                    sum_val = 0;
                    if j > 1
                        for k = 1:j-2
                            sum_val = sum_val + APP0(i,k) * APP0(j,k);
                        end
                    end
                    if APP0(j,j) == 0
                        APP0(i,j) = 0;
                    else
                        APP0(i,j) = (PP0(i,j) - sum_val) / APP0(j,j);
                    end
                elseif j == i
                    sum_val = 0;
                    if i > 1
                        for k = 1:i-2
                            sum_val = sum_val + APP0(i,k)^2;
                        end
                    end
                    APP0(i,j) = sqrt(PP0(i,i) - sum_val);
                else
                    APP0(i,j) = 0;
                end
            end
        end

    % Draw Gaussian noise vector
    GAUSS = randn(9,1);  % standard normal (mean=0, std=1)

    % Multiply: XX0 = APP0 * GAUSS
    XX0 = APP0 * GAUSS;

    % Form the initial error vectors
    ESBI = XX0(1:3) * (1 + frax);       % m
    EVBI = XX0(4:6) * (1 + frax);       % m/s
    RICI = XX0(7:9) * (1 + frax) * 1e-3; % mrad → rad

end
