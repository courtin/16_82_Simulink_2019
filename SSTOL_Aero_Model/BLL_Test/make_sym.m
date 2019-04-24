function [sym_array] = make_sym(array, ax)
%If array is 3xN, flip the sign of the row specified by ax
%i.e. ax = 2 specifies y-symmetry
%If array is 1xN, assume no sign change

s =size(array);

array_sym = array;

if s(1) == 3
    array_sym(ax,:) = array_sym(ax,:)*-1;
    sym_array = [fliplr(array_sym), array];
else
    sym_array = [fliplr(array_sym), array];
end
end