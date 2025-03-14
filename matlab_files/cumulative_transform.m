

function [outputArg1] = cumulative_transform(varargin)
%   INPUT:
%       varargin - any number of 4x4 matrices
%   OUTPUT:
%       outputArg1 - cell array where each entry contains the cumulative 
%                    multiplication result up to that matrix in sequence.

    n= nargin;
    res=cell(1,n);

    cum_product= eye(4);

        % Check that all inputs are 4x4 matrices
    for i = 1:n
        if ~ismatrix(varargin{i}) || ~isequal(size(varargin{i}), [4, 4])
            error('All inputs must be 4x4 matrices.');
        end
    end

     for i = 1:n
        cum_product = cum_product * varargin{i}; % Multiply with next matrix
        cum_product=simplify(cum_product);
        res{i} = cum_product; % Store the result in the cell array
     end
    outputArg1=res;
end

