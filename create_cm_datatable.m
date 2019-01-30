function cms = create_cm_datatable(cm_fit,fits)

cms = zeros(length(fits.alpha_range),length(fits.dCJ_range),length(fits.flaps_range));

for i = 1:length(fits.alpha_range)
    for j = 1:length(fits.dCJ_range)
        for k = 1:length(fits.flaps_range)
            cms(i,j,k) = cm_fit(fits.alpha_range(i),fits.dCJ_range(j));
        end
    end
end
