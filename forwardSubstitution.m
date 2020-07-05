function x=forwardSubstitution(L,b,n)
% Solving a lower triangular system by forward-substitution
% Input matrix L is an n by n lower triangular matrix
% Input vector b is n by 1
% Input scalar n specifies the dimensions of the arrays
% Output vector x is the solution to the linear system
% L x = b
% K. Ming Leung, 01/25/03

x=zeros(n,1);
for j=1:n
    if (L(j,j)==0) error('Matrix is singular!'); end;
    x(j)=b(j)/L(j,j);
    b(j+1:n)=b(j+1:n)-L(j+1:n,j)*x(j);
end 

end