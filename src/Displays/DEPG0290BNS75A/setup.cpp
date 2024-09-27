#include "DEPG0290BNS75A.h"

void DEPG0290BNS75A::init() {
    BaseDisplay::panel_width = this->panel_width;
    BaseDisplay::panel_height = this->panel_height;

    BaseDisplay::supported_colors = this->supported_colors;

    // Not set for this display: same as panel
    BaseDisplay::drawing_width = this->panel_width;
    BaseDisplay::drawing_height = this->panel_height;

    // Get the Bounds subclass ready now (in constructor), so that it can be used to init. globals.
    BaseDisplay::instantiateBounds();
    
     // Set the initial drawing config early, so user's config commands before the first update aren't ignored
    BaseDisplay::initDrawingParams(); 
}