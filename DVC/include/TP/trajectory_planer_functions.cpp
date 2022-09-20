#include <trajectory_planer_functions.h>

#include <algorithm>


#define MODE 0
// 0 for localmode 1 for gps mode

object_global_localizator_msgs::ObjectsGlobalPositions objGlobPos;
bool readFlag;

std::vector<TreeObejctPosition> treePosVec;
bool trajectoryRecalculateFlag;
Point droneOldPos(0,0);
size_t goalPointId;
bool goolFlag;


sensor_msgs::NavSatFix global_position;
nav_msgs::Odometry local_position;


ros::Publisher goal_pos_pub;

trajectory_planer_msgs::TrajectoryPlaner achievePos;
bool readAchievePos;



void new_Point_cb(const object_global_localizator_msgs::ObjectsGlobalPositions::ConstPtr& msg){
    objGlobPos = *msg;
    readFlag = true;
    //ROS_INFO("global pos read");
}

void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_position = *msg;
    //ROS_INFO("global pos read");
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    local_position = *msg;
    //ROS_INFO("local pos read");
}

void achieve_point_cb(const trajectory_planer_msgs::TrajectoryPlaner::ConstPtr& msg){
    achievePos = *msg;
    readAchievePos = true;
    //ROS_INFO("local pos read");
}


bool tree_table_cb(trajectory_planer_msgs::treeTable::Request& req, trajectory_planer_msgs::treeTable::Response& res)
{
    if(req.sendResponse == true)
    {
        for(const auto& treePos:treePosVec)
        {
            trajectory_planer_msgs::SimpleTree tree;

            tree.idClassObject = treePos.getId();
            tree.updateCounter = treePos.getUpdateCounter();

            Point p = treePos.getPoint();

            tree.pos1 = p.getPos1();
            tree.pos2 = p.getPos2();

            res.treeTable.push_back(tree);
        }
    }
    return true;
}

void resetReadFlag()
{
    readFlag = false;
    readAchievePos = false;

}

void resetGoolFlag()
{
    goolFlag = false;
}

bool checkReadFlag()
{
    return readFlag;
}

void processReadPoints()
{
    for (const auto & pointObj : objGlobPos.ObjectsGlobalPositions) //wykonaj czynnosci dla wszystkich obiektow odebranych z object_global_localizator
    {
        unsigned short id = pointObj.idClassObject;  // pobierz id obiektu
        double distance = pointObj.distanceDroneToObject; // pobierz odleglosc do obiektu

#if MODE == 0
        Point p (pointObj.globalPositionLocal.x, pointObj.globalPositionLocal.y); // stworz punkt p o wspolrzednych lokalnych
#else
        Point p (pointObj.latitude, pointObj.longitude); // stworz punkt p o wspolrzednych GPS
#endif

        bool succes = false;
        for(auto& treePos:treePosVec) //wykonaj czynnosci dla wszystkich zapisanych obiektow
        {
            //ROS_INFO("id: %d v id: %d",id,treePos.getId());
            //ROS_INFO("x: %f v x: %f",p.getPos1(),treePos.getPoint().getPos1());
            //ROS_INFO("y: %f v y: %f",p.getPos2(),treePos.getPoint().getPos2());
            if(treePos.addIfInclude(p,id,1.0/distance))
            {
                //ROS_INFO("add id: %d p1: %f p2: %f",id,p.getPos1(),p.getPos2());
                succes = true;
                break;
            }
        }

        if(!succes) //jesli obiekt nie zostal wczesniej zapisany
        {

#if MODE == 0
            double radius = 2.0;
#else
            double radius = 0.001;
#endif

            TreeObejctPosition top (id,p,radius,1.0/distance); //stworz nowy obiekt
            treePosVec.push_back(top); //dodaj do listy obiektow nowy obiekt
            trajectoryRecalculateFlag = true; // ustaw flage do ponownego obliczenia trajektorii

            Point p1 = top.getPoint(); //pobierz wspolrzene nowego obiektu
            ROS_INFO("new id: %d p1: %f p2: %f",id,p1.getPos1(),p1.getPos2());
            ROS_INFO("new id: %d p1: %f p2: %f",id,p.getPos1(),p.getPos2());
        }

    }
}

void findTrajectory(ros::NodeHandle controlNode)
{
    int idToGo1, idToGo2, idToGo3;
    controlNode.getParam("/trajectory_planer/idToGo1", idToGo1); //pobierz id wykrywanego obiektu
    controlNode.getParam("/trajectory_planer/idToGo2", idToGo2); //pobierz id wykrywanego obiektu
    controlNode.getParam("/trajectory_planer/idToGo3", idToGo3); //pobierz id wykrywanego obiektu
#if MODE == 0
    // pozycja dona na podstawie wspolrzednych lokalnych
    Point dronePos(local_position.pose.pose.position.x,local_position.pose.pose.position.y);
#else
    // pozycja drona na podstawie wspolrzednych GPS
    Point dronePos(global_position.latitude,global_position.longitude);
#endif

    //jesli pozycja drona wzgledem poprzedniej jest wieksza lub rozna niz promien wykrywania obiektu
    if(dronePos.countDistance(droneOldPos)>= pow(treePosVec[0].getRadius(),2))
    {
        droneOldPos = dronePos; //przypisz aktualna pozycje jako stara
        trajectoryRecalculateFlag = true; // ustaw flage do ponownego obliczenia trajektorii
    }


    //jesli flaga do ponownego obliczenia jest ustawiona
    if(trajectoryRecalculateFlag)
    {
        trajectoryRecalculateFlag = false; //ustaw flage by nie obliczac ponownie trajektorii

        std::vector<Point> points; //stworz liste punktow


        for(const auto& treePos:treePosVec) //wykonaj czynnosci dla wszystkich znalezionych obiektow
        {
            //jesli id obiektu jest rowne temu ktore ma byc obsluzone
            if((treePos.getId() == idToGo1 || treePos.getId() == idToGo2 || treePos.getId() == idToGo3) && !treePos.isVisited())
            {
                points.push_back(treePos.getPoint()); //dodaj punkt do listy punktow
            }
        } 

        //jesli lista punktow jest pusta
        if (points.size()==0) {
            goolFlag = false; //ustaw flage by nie wysylac kolejnego waypoint'u
            return;
        }

        //storz liste

        std::vector<size_t> trajectory = findBestTrajectory(points,dronePos);

        Point goalPoint = points[trajectory[0]];
        for(size_t i = 0; i < treePosVec.size(); i++)
        {
            if(goalPoint.countDistance(treePosVec[i].getPoint())<= pow(treePosVec[0].getRadius(),2))
            {
                goalPointId = i;
                break;
            }
        }

        goolFlag = true;

    }


}

std::vector<size_t> findBestTrajectory(const std::vector<Point>& points, const Point& dronePos)
{

    std::vector<size_t> result; //stworz liste
    double minCost = 1e300;

    for(size_t i = 0; i < points.size(); i++) //wykonaj czynnosci tyle razy ile wynosi liczba punktow
    {
        Point next = points[i]; //stworz punkt do analizy
        std::vector<size_t> v = {i}; //stworz liste do ktorej bedzie zapisywana kolejnosc lotu

        //wylicz koszt zalezny od polozenia drona wgledem analizowanego punktu
        double cost = next.countDistance(dronePos);

        findLoverCost(points,v,cost,minCost,result);
    }

    return result;

}

void findLoverCost (const std::vector<Point>& points, std::vector<size_t>& v, double actualCost, double& minCost, std::vector<size_t>& result)
{
    //jesli liczba sprawdzonych punktow jest rowna liczbie wytypowanych obiektow
    if(v.size() == points.size())
    {
        //jesli aktualny koszt jest mniejszy niz koszt poprzedni
        if(actualCost<minCost)
        {
            result = v; //przypisz kolejnosc o mnniejszym koszcie jako rezultat
            minCost = actualCost; //przypisz aktualny kosz jako koszt optymalny
        }
    }
    else //jesli warunek jest nie spelniony
    {
        //wykonaj czynnosci tyle razy ile wynosi liczba punktow
        for(size_t i = 0; i < points.size(); i++)
        {

            //jesli nie znajdzie w zbiorze v wartosci i

            if(std::find(v.begin(), v.end(), i) == v.end())
            {
                Point last = points[*(v.end()-1)]; //przypisz punkt jako ostatni ze zbioru v
                Point next = points[i]; //przypisz kolejny punkt jako kolejny punkt z listy punktow

                //dodal do aktualnego kosztu wartosc pomiedzy aktualnie wybranym punktem a ostatnio wytypowanym punktem
                actualCost += next.countDistance(last);
                v.push_back(i); //dodaj do listy v wartos i

                findLoverCost(points,v,actualCost,minCost,result);

            }
        }
    }
}

void init_publisher(ros::NodeHandle controlNode){
    goal_pos_pub = controlNode.advertise<trajectory_planer_msgs::TrajectoryPlaner>("/trajectory_planer/next_waypoint", 1);
}

void sendOutMessage()
{
    //jesli jest ustawiona flaga do wysania wiadomosci
    if(goolFlag) {
        trajectory_planer_msgs::TrajectoryPlaner outMessage; //stworz wiadomosc

#if MODE == 0 // jesli mode jest jako lokal
        outMessage.mode = "local"; // przypisz wartosc mode jako lokalna
#else
        outMessage.mode = "global"; // przypisz wartosc mode jako global
#endif

        Point p = treePosVec[goalPointId].getPoint(); //stworz punkt o wspolrzednych wytypowanego obiektu

        outMessage.pos1 = p.getPos1(); //przypisz pozycje 1 wytypowanego obiektu do wiadomosci
        outMessage.pos2 = p.getPos2(); //przypisz pozycje 2 wytypowanego obiektu do wiadomosci

        outMessage.idClassObject = treePosVec[goalPointId].getId(); //przypisz id obiektu do wiadomosci
        outMessage.updateCounter = treePosVec[goalPointId].getUpdateCounter(); //przypisz ilosc wykrytych razy obiektu do wiadomosci

        //ROS_INFO("Goal p1: %f p2: %f", p.getPos1(), p.getPos2());

        goal_pos_pub.publish(outMessage); //publikuj wiadomosc
    } else{
        trajectory_planer_msgs::TrajectoryPlaner outMessage; //stworz wiadomosc
        outMessage.mode = "empty"; // przypisz wartosc mode jako pusty
        goal_pos_pub.publish(outMessage); // publikuj wiadomosc
    }
}

void printInfo()
{
    //jesli jest ustawiona flaga do wysania wiadomosci
    if(goolFlag)
    {
        ROS_INFO("Actual Points");
        for(const auto& treePos:treePosVec) //wykonaj czynnosci dla wszystkich wykrytych obiekt
        {
            //stworz punkt o wspolrzednych badanego obiektu
            Point p = treePos.getPoint();
            ROS_INFO("ID %d Goal p1: %f p2: %f",treePos.getId(), p.getPos1(), p.getPos2());

        }
        ROS_INFO("Goal has index %d",int(goalPointId));
        Point p = treePosVec[goalPointId].getPoint(); //stworz punkt o wspolrzednych wybranego obiektu
        ROS_INFO("Goal p1: %f p2: %f", p.getPos1(), p.getPos2());
    }
}


void setVisitedPoint()
{
    //jesli pozycja zostala osiagnieta (informacja pobrana z topic'a)
    if(readAchievePos)
    {
        readAchievePos=false; //zmien stan falgi
        Point p (achievePos.pos1, achievePos.pos2); //stworz punkt o wsporzednych punktu osiagnietego

        for(auto& treePos:treePosVec) //wykonaj czynnosc dla kazdego wykrytego obiektu
        {
            //jesli  dystans pomiedzy osiagnietym punktem a badanym obiektem jest mniejszy niz promien dzialania
            if(p.countDistance(treePos.getPoint())<= pow(treePosVec[0].getRadius(),2))
            {
                treePos.setVisited(); //ustaw flage ze obiekt zostal odwiedzony
            }
        }
        trajectoryRecalculateFlag = true; // ustaw flage do ponownego obliczenia trajektorii
    }
}