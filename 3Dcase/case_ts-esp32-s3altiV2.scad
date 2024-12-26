/****************************************************************
* title: Alti case TS-ESP32-S3
* author: Boris du Reau
* version: 1.0
* date: 26/12/2024
* 
* Description: 
* Howto use it:
* Modify the variables values so that it fits to your rocket tube 
* then just click on "compile and render" 
* Note that all units ar metrics
*****************************************************************/

/* adjust variables here */ 

/* Do not modify anything after that unless you know what you are doing*/

module roundedcube(xdim, ydim, zdim,rdim) {
    hull(){
        translate([rdim, rdim,0]) cylinder(r=rdim, h=zdim);
        translate([xdim-rdim, rdim,0]) cylinder(r=rdim, h=zdim);
        translate([rdim, ydim -rdim,0]) cylinder(r=rdim, h=zdim);
        translate([xdim-rdim, ydim-rdim,0]) cylinder(r=rdim, h=zdim);
    }
}


module box() {
difference() {
    union() {
        difference() {
            union() {
                translate([0,0,-2])roundedcube(23 +(2*1.6), 54+ (2*1.6), 12,3); 
                difference() {
                    #translate ([(23 +(2*1.6))/2,54+ (2*1.6),4]) #cylinder(r=26/2, h=6, $fn=100);
                    #translate ([(23 +(2*1.6))/2,54+ (2*1.6),4]) #cylinder(r=16/2, h=6, $fn=100);
                }
            } 
            #translate ([1.6,1.6,-2]) roundedcube(23, 54, 10,2);  
        }     
        
        //case holes
        //#translate([0+2,2,4])cylinder(r=2, h=12, center = true, $fn =100);
        //#translate([0+28,2,4])cylinder(r=2, h=12, center = true, $fn =100);
        #translate([0+2,58-2,4])cylinder(r=2, h=12, center = true, $fn =100);
        #translate([0+24,58-2,4])cylinder(r=2, h=12, center = true, $fn =100); 
        
        //board holes
        #translate([0+2+2.5,2+3,6])cylinder(r=2, h=7, center = true, $fn =100);
        #translate([0+2+1.5+18,2+3,6])cylinder(r=2, h=7, center = true, $fn =100);
    }
    //case holes
    //#translate([0+2,2,0])cylinder(r=2/2, h=30, center = true, $fn =100);
    //#translate([0+28,2,0])cylinder(r=2/2, h=30, center = true, $fn =100);
    #translate([0+2,58-2,0])cylinder(r=2/2, h=30, center = true, $fn =100);
    #translate([0+24,58-2,0])cylinder(r=2/2, h=30, center = true, $fn =100);
    //board holes
    #translate([0+2+2.5, 2+3,5])cylinder(r=2/2, h=7, center = true, $fn =100);
    #translate([0+2+1.5+18, 2+3,5])cylinder(r=2/2, h=7, center = true, $fn =100);
    
    //usb port
    #translate([(23 +(2*1.6))/2,1,0])cube([10,2,4], center = true);
    //sensor hole
    #translate([20,25,4])cylinder(r=3, h=12, center = true, $fn =100);
}
    
}

module cover() {
difference() {
    union() {
        difference() {
            translate([0,0,0])roundedcube(23 +(2*1.6), 54+ (2*1.6), 2,3);     
        }
        //case holes
        //#translate([0+2,2,1])cylinder(r=2, h=2, center = true, $fn =100);
        //#translate([0+28,2,1])cylinder(r=2, h=2, center = true, $fn =100);
        #translate([0+2,55,1])cylinder(r=2, h=2, center = true, $fn =100);
        #translate([0+24,55,1])cylinder(r=2, h=2, center = true, $fn =100);
    }
    
    //case holes
    #translate([0+2,55,0])cylinder(r=2/2, h=30, center = true, $fn =100);
    #translate([0+24,55,0])cylinder(r=2/2, h=30, center = true, $fn =100);
    //board holes
    #translate([0+2+2.5,2+3,0])cylinder(r=2/2, h=30, center = true, $fn =100);
    #translate([0+2+1.5+18,2+3,0])cylinder(r=2/2, h=30, center = true, $fn =100);
    
    //window
    #translate([(23 +(2*1.6))/2,17.5+(31/2),2]) cube([18, 28, 4], center = true);
    #translate([(23 +(2*1.6))/2,17.5+(31/2),0])resize([22,32])rotate([0,0,45])cylinder(h=2, r1= 20, r2 = 32, $fn=4);
    
    #translate([2.5,9.5,0]) cube([7.5,9,4]);
    #translate([11,10.5,0]) cube([5,8,4]);
    #translate([17,14,0]) cube([5,4.5,4]);
} 
    
}

//box();
cover();
