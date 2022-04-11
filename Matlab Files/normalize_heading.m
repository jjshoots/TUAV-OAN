function HEADING = normalize_heading(HEADING1, HEADING2, MAG1, MAG2)
    if MAG1 == 0 && MAG2 > 0
        HEADING = HEADING2 / norm(HEADING2);
    elseif MAG1 > 0 && MAG2 == 0
        HEADING = HEADING1 / norm(HEADING1);
    elseif MAG1 == 0 && MAG2 == 0
        HEADING = HEADING1 / norm(HEADING1);
    else
        HEADING = HEADING1./MAG1 + HEADING2./MAG2;
        HEADING = HEADING / norm(HEADING);
    end
end