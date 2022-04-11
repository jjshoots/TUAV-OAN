function ARB = max_ARB(new_ARB, old_ARB)
    if new_ARB > old_ARB
        ARB = new_ARB;
    else
        ARB = old_ARB;
    end
end