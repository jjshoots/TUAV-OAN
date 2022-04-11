thesis_var;
for a = 1:size(thesis_OBSRAD, 2)
    for b = 1:size(thesis_PROX, 2)
        for c = 1:size(thesis_MAXVEL, 2)
            for d = 1:size(thesis_ATC, 2)
                for e = 1:size(thesis_WRT, 2)
                    for f = 1:size(thesis_SVT, 2)
                        clearvars -except a b c d e f;
                        DNF = 0;
                        variables;
                        thesis_var;
                        
                        ZVAR_OBS_RAD = thesis_OBSRAD(a);
                        ZVAR_PROX = thesis_PROX(b);
                        ZVAR_MAX_VELO = thesis_MAXVEL(c);
                        ACCE_GAIN = 1 / thesis_ATC(d);
                        ZVAR_WRT = thesis_WRT(e);
                        ZVAR_THRESHOLD = thesis_SVT(f);
                        dat1 = [ZVAR_OBS_RAD ZVAR_PROX ZVAR_MAX_VELO ACCE_GAIN ZVAR_WRT ZVAR_THRESHOLD];
                        
                        test;                        
                        
                        if ~DNF
                            thesis_data;
                            dat2 = [TOTAL_TIME MAX_ARB NUM_A_STAR MAX_JERK AVE_JERK];
                            total_dat = [dat1 dat2];
                        else
                            dat2 = [-1 -1 -1 -1 -1]
                            total_dat = [dat1 dat2];
                        end
                        
                        fid = fopen('A:\Users\Tai\Desktop\FYP_Matlab\the_data\dat.csv', 'a+');
                        fprintf(fid, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n', total_dat);
                        fclose(fid);
                    end
                end
            end
        end
    end
end
