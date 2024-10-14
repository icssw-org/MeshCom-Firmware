#include "DEPG0213BNS800.h"

void DEPG0213BNS800::init() {
    // Pass information to base class, once instantiated
    BaseDisplay::panel_width = this->panel_width;
    BaseDisplay::panel_height = this->panel_height;

    BaseDisplay::drawing_width = this->drawing_width;
    BaseDisplay::drawing_height = this->drawing_height;

    BaseDisplay::supported_colors = this->supported_colors;

    // Get the Bounds subclass ready now (in constructor), so that it can be used to init. globals.
    BaseDisplay::instantiateBounds();
    
     // Set the initial drawing config early, so user's config commands before the first update aren't ignored
    BaseDisplay::initDrawingParams();     
}