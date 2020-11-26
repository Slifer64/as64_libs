function [is_spd, e] = isSPD(A)

    e = eig(A);
    is_spd = isempty(find(e<=0));

end