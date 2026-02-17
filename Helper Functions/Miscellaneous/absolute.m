function ret = absolute(vec)
    % Computes the Euclidean norm (magnitude) of a vector
    %
    % Input:
    %   vec - Column or row vector (Nx1 or 1xN)
    %
    % Output:
    %   ret - Magnitude of the vector (scalar)

    % Compute the Euclidean norm (L2 norm)
    ret = sqrt(sum(vec.^2));
end
