include <MCAD/units/metric.scad>
use <MCAD/array/along_curve.scad>
use <MCAD/shapes/cylinder.scad>
use <scad-utils/transformations.scad>
use <scad-utils/shapes.scad>
use <MCAD/general/sweep.scad>
use <MCAD/shapes/2Dshapes.scad>

echo ("hub to tip ratio: ", hub_d/propeller_d*100);


number_of_blades = 3;
wall_thickness = .8;
hub_d = 63;
hub_od = hub_d + wall_thickness * 2;
hub_recess = 60.0;
hub_motor = 18.0;
propeller_d = 93.5;

blade_height = 33;
blade_thickness = 0.3;
blade_pitch = 60;               // average, because it's parabolic
blade_direction = -1;           // -1 for counter-clockwise, 1 for clockwise
blade_shape = "parabolic";      // "parabolic" or "linear"

winglets = false;
winglet_thickness = 0.5;
winglet_length = 5;
winglet_direction = 1;          // 1 for underside of blade, -1 for upper

shroud = false;
shroud_thickness = 2;
cowl_height=30;
cowl_offset=20;

tab_internal_angle = 10;
tab_thickness = 0.2;
number_of_tabs = 5;
tab_width = 0.2;
hub_hole = 4.0;
screwhole_r = 0;

$fs = 0.4;
$fa = 1;
difference() {
    union() {
        threaded_bolt_hub();
        difference() {
            mcad_rotate_multiply (number_of_blades, axis = Z)
            translate ([0- epsilon, 0, 0])
            blade ();
            threaded_bolt_hub_no_central();
        }
        if (shroud)
        shroud ();
    }
    translate([0,0,-0.1]) cylinder(d=hub_recess, h=hub_motor+0.1);
}

module threaded_bolt_hub() {
    difference () {
      threaded_bolt_hub_no_central();
        for (i=[0:90:360]) {    //screw holes
            rotate([0,0,i]) translate([screwhole_r,0,-1]) cylinder(d=hub_hole, h=70);
        }
    }
}

module threaded_bolt_hub_no_central() {
    union() {
        cylinder(d=hub_od, h=hub_motor);
        translate([0,0,hub_motor]) cylinder(d1=hub_od, d2=hub_od/2, h=blade_height-hub_motor);
        translate([0,0,blade_height]) cylinder(d1=hub_od/2, h=10);
    }
}

module tab ()
{
    linear_extrude (height = tab_thickness) {
        intersection () {
            difference () {
                circle (d = hub_od);
                circle (d = hub_od - (tab_width + wall_thickness) * 2);
            }

            pieSlice (hub_od, -tab_internal_angle / 2, tab_internal_angle / 2);
        }
    }
}

module blade ()
{
    lead_r = (propeller_d - hub_od) / 2;
    blade_angle = tan (90 - blade_pitch) * blade_height / (PI * 2 * lead_r) * 360;

    function parabolic_twist (t) = ((pow (t, 2) + 0.5 * t) * blade_angle *
        blade_direction);
    function linear_twist (t) = (t * blade_angle * blade_direction);
    
    function nema_twist (t) = (t* blade_angle * blade_direction);

    function twist (t) = (
       // (blade_shape == "parabolic") ? parabolic_twist (t) :
       // linear_twist (t)
        nema_twist(t)
        //parabolic_twist(t)
    );

    blade_length = propeller_d / 2;
    base_shape = rectangle_profile ([blade_length,
            tan (blade_pitch) * blade_thickness]);
    blade_transforms = [
        for (t = [0 : 0.01 : 1.005])
        rotation ([0, 0, twist (t)]) *
        translation ([blade_length / 2, 0, (1 - t) * blade_height])
    ];

    winglet_shape = rectangle_profile ([winglet_thickness, winglet_length]);
    winglet_transforms = [
        for (i = [0 : len (blade_transforms) - 1])
        let (
            transform = blade_transforms[i],
            scale_ = max (0.01,
                (len (blade_transforms) - i) / len (blade_transforms)
            )
        )
        transform *
        translation ([
                -winglet_thickness / 2 + blade_length / 2,
                (scale_ * winglet_length / 2 * winglet_direction *
                    blade_direction),
                0
            ]) *
        scaling ([1, scale_, 1])
    ];
  //  render ()
        union () {
            sweep (base_shape, blade_transforms);
            if (winglets)
            sweep (winglet_shape, winglet_transforms);
        }
}

module shroud ()
{
    rotate_extrude ()
    translate ([shroud_thickness / 2 + (propeller_d / 2 + 1), -cowl_offset])
    square ([shroud_thickness, blade_height+cowl_height]);
}