#include <TreeObejctPosition.h>


TreeObejctPosition::TreeObejctPosition(unsigned short id, Point sum, double radius, double num) : id(id), sum(sum),  radius(radius), num(num), visited(0), updateCounter(0) {
    this->sum.setPos(sum.getPos1() * num, sum.getPos2() * num); //wspolrzedne puktow z infomacja o odleglosci od drona
    this->id_list.push_back(this->id);
}

bool TreeObejctPosition::addIfInclude (const Point& p, unsigned short idp, double weigth)
{

    if(ifInclude(p)) //jesli nowy punkt nalezy do obiektu i id obiektu sie zgadza z nowo wykrytym
    {
        //dodaj nowa pozycje z infomacja o polozeniu od drona do reszty punktow
        sum.setPos(sum.getPos1()+(p.getPos1()*weigth),sum.getPos2()+(p.getPos2()*weigth));
        num+=weigth; //zwieksz wspolna wage o wage nowego punktu
        updateCounter++; //zwieksz licznik iformujacy ile razy dany obiekt by obserwowany

        id_list.push_back(idp);
        size_t number_of_id1 = 0;
        size_t number_of_id2 = 0;
        size_t number_of_id3 = 0;

        for(int i = 0; i < id_list.size(); i++)
        {
            if(id_list[i]==1)
            {
                number_of_id1++;
            }
            if(id_list[i]==2)
            {
                number_of_id2++;
            }
            if(id_list[i]==3)
            {
                number_of_id3++;
            }
        }

        id_list.push_back(idp); 

        if(number_of_id1 > number_of_id2 && number_of_id1 > number_of_id3)
        {
            id=1;
        }
        if(number_of_id2 > number_of_id1 && number_of_id2 > number_of_id3)
        {
            id=2;
        }
        if(number_of_id3 > number_of_id2 && number_of_id3 > number_of_id1)
        {
            id=3;
        }
        if(number_of_id1==number_of_id2 || number_of_id1==number_of_id3)
        {
            id=1;
        }
        if(number_of_id2==number_of_id3)
        {
            id=2;
        }
        return true; //zwroc prawde
    }
    return false; //jesli warunki nie sa spelnione zwroc falsz
}

bool TreeObejctPosition::ifInclude (const Point& p)
{
    Point mainP = getPoint(); //pobierz wspolrzedne punktu bez informacji o odleglosci od drona

    return (pow(p.getPos1()-mainP.getPos1(),2) + pow(p.getPos2()-mainP.getPos2(),2)) <= pow(radius,2);
    // zwroc informacje czy nowy punkt miesci sie w zasiegu wykrytego obiektu
}

Point TreeObejctPosition::getPoint()const
{
    return Point(sum.getPos1()/num, sum.getPos2()/num); // zwroc punkt o wspolrzednych
}