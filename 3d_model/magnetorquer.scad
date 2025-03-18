inductor_h = 70;
inductor_r = 10;

endcap_h = 10;
endcap_r = 20;
endcap_flat_r = 16; // radius until the flat part
// heat insert
heat_insert_r = 4.1 / 2;
heat_insert_depth = 5.9;
heat_insert_thr_r = 3 / 2;

buffer = 10;

holder_h = inductor_h + endcap_h * 2;

rotate(a = 90, v = [ 1, 0, 0 ]) {

  group() {
    cylinder(h = inductor_h,
             r1 = inductor_r,
             r2 = inductor_r,
             center = true);
    translate(v = [ 0,
                    0,
                    inductor_h / 2 + endcap_h / 2 ]) {
      difference() {
        cylinder(h = endcap_h,
                 r1 = endcap_r,
                 r2 = endcap_r,
                 center = true);
        // cut out
        mirror(v = [ 0, 1, 0 ]) {
          translate(v = [ 0,
                          endcap_r - (endcap_r - endcap_flat_r) / 2,
                          0 ]) {
            cube(size =
                     [
                       endcap_r * 2, endcap_r - endcap_flat_r, endcap_h + buffer
                     ],
                 center = true);
          }
        }
        // heat insert
        translate(
            v = [ 0, -endcap_flat_r + heat_insert_depth / 2 - buffer / 2, 0 ]) {
          rotate(a = 90, v = [ 1, 0, 0 ]) {
            group() {
              cylinder(h = heat_insert_depth + buffer, r1 = heat_insert_r,
                       r2 = heat_insert_r, center = true);
              translate(v = [ 0, 0, -10 ]) {
                cylinder(h = heat_insert_depth + buffer, r1 = heat_insert_thr_r,
                         r2 = heat_insert_thr_r, center = true);
              } // end translate
            } // end group
          } // end rotate
        } // end translate
        // end heat insert
      } // end difference
    } // end translate
    mirror(v = [ 0, 0, 1 ]) {
      translate(v = [ 0, 0, inductor_h / 2 + endcap_h / 2 ]) {
        difference() {
          cylinder(h = endcap_h, r1 = endcap_r, r2 = endcap_r, center = true);
          // cut out
          mirror(v = [ 0, 1, 0 ]) {
            translate(v = [ 0, endcap_r - (endcap_r - endcap_flat_r) / 2, 0 ]) {
              cube(size =
                       [
                         endcap_r * 2, endcap_r - endcap_flat_r, endcap_h +
                         buffer
                       ],
                   center = true);
            }
          }
          // heat insert
          translate(v = [
            0, -endcap_flat_r + heat_insert_depth / 2 - buffer / 2, 0
          ]) {
            rotate(a = 90, v = [ 1, 0, 0 ]) {
              group() {
                cylinder(h = heat_insert_depth + buffer, r1 = heat_insert_r,
                         r2 = heat_insert_r, center = true);
                translate(v = [ 0, 0, -10 ]) {
                  cylinder(h = heat_insert_depth + buffer,
                           r1 = heat_insert_thr_r, r2 = heat_insert_thr_r,
                           center = true);
                } // end translate
              } // end group
            } // end rotate
          } // end translate
          // end heat insert
        } // end difference
      } // end translate
    } // end mirror
  } // end group
} // end rotate
