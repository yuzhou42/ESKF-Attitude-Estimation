function chi2 = calChi2(angleDiff)
angleDiff2 = angleDiff.^2;
len = size(angleDiff,1);
E   = sum(angleDiff)/len;
E2  = sum(angleDiff2)/len;
var = E2-E^2;

end