// --------------------------------------------------------------------------
//
//  Copyright (C) 2011, 2017 Matthias Lutz, Dennis Stampfer, Matthias Rollenhagen, Nayabrasul Shaik
//
//      lutz@hs-ulm.de
//      stampfer@hs-ulm.de
//      rollenhagen@hs-ulm.de
//      shaik@hs-ulm.de
//
//      ZAFH Servicerobotic Ulm
//      Christian Schlegel
//      University of Applied Sciences
//      Prittwitzstr. 10
//      89075 Ulm
//      Germany
//
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//--------------------------------------------------------------------------
#include "ImageQueryHandler.hh"
#include "ComponentWebots3DCamera.hh"
#include <chrono>
#include <thread>

ImageQueryHandler::ImageQueryHandler(
    Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> *server) :
    ImageQueryHandlerCore(server) {

}

// see ColorImageQueryHandler.cc
void ImageQueryHandler::handleQuery(const Smart::QueryIdPtr &id, const CommBasicObjects::CommVoid &request) {

    DomainVision::CommRGBDImage answer;
    answer.setIs_valid(false);

    if (COMP->stateSlave->tryAcquire("nonneutral") == Smart::SMART_OK) {
        if (COMP->stateSlave->tryAcquire("pushimage") == Smart::SMART_OK) {
            COMP->NewestImageMutex.acquire();
            if (COMP->newestImage != NULL) {
                answer = *(COMP->newestImage);
            }
            COMP->NewestImageMutex.release();
            COMP->stateSlave->release("pushimage");
        } else {
            bool imageFound = false;
            while (!imageFound) {
                COMP->imageTask->isQueryImage = true;
                COMP->NewestImageMutex.acquire();
                if (COMP->newestImage != NULL) {
                    answer = *(COMP->newestImage);
                    imageFound = true;
                    COMP->newestImage = NULL;
                }
                COMP->NewestImageMutex.release();
                if(!imageFound)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            COMP->imageTask->isQueryImage = false;
        }
        COMP->stateSlave->release("nonneutral");
    }
    server->answer(id, answer);
    return;
}
