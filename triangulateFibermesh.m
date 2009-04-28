y = [ 297 297 297 297 297 297 297 297 297 297 297 297 297 ]; x = [ 307 302 300 298 290 288 284 305 301 297 293 289 285  ]; t = delaunay(x,y); save('triangulateFibermesh.mat', 't', '-ASCII'); quit;
