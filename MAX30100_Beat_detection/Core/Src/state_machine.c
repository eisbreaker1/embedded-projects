#include "state_machine.h"


/*! \var char output[256]
 * \brief Bufor ze znakami zapisanymi za pomoca funkcji wwrite
 */
char output[256];
/*! \var int out_pos
 * \brief Biezaca pozycja do zapisu w buforze output
 */
int outpos=0;




/*! \var char mess[32]
 * \brief Bufor ze znakami do tymczasowego zapisu znakow odczytywanych za pomoca funkcji rread
 */
char mess[64];
/*! \var int mess_pos
 * \brief Biezaca pozycja do zapisu w buforze mess
 */
int mess_pos=0;





/*! \var char buf[128]
 * \brief Bufor z symulowanymi pakietami znakow
 */

char buf[129]=    "1\\1 Ala ma kota 0Przypadkowe dane12 Kot ma Ale 0Przypadkow1e dane13 Mniej niz \\0 0Przypadkowe dane14 TAK\\\\NIE 0Przypadkowe dane0";
// (-1-) Przed testem nowej maszyny zakomentuj linie powyzej i odkomentuj linie ponizej
//char buf[129]=    "1\\1 Ala ma kota 0Przypadkowy ciag znakow12 Kot ma Ale 013 Zbyt dlugi ciag znakow powoduje blad 014 Mniej niz \\0 0Przykladowy1 ci";

/*! \var int pos
 * \brief Biezaca pozycja do odczytu z bufora buf
 */
int pos=127;






/*! \fn char rread()
 * \brief Atrapa funkcji symulujaca odczyt danej z portu komunikacyjnego
 * \return Funkcja zwraca znak z ci�gu znakow zawartych w zmiennej globalnej buf
 */
char rread()
{
	pos=(pos+1)&127;
	return buf[pos];
}





/*! \fn void wwrite(char* data, int len)
 * \brief Funkcja zapisujaca ciag znakow do zmiennej output
 * \param data tablica znakow do zapisania w buforze output
 * \param len ilosc znakow zapisania w buforze output
 */
void wwrite(char* data, int len)
{
	int i;
	for(i=0;i<len;++i)
	{
		output[outpos]=data[i];
		outpos=(outpos+1)&255;
	}
}




/*! \fn void delay()
 * \brief Funkcja opozniajaca realizowana w postaci petli opozniajacej
 */
void delay()
{
	int i;
	for(i=0;i<5e5;i++);
}






/*! \fn void signal_success()
 * \brief Funkcja sygnalizujaca diodami odczytanie prawidlowego pakietu danych
 */
void signal_success()
{
	unsigned char c=0;
	for(c=0;c<4;c++)
	{
		LED(1<<c);
		delay();
	}
}






/*! \fn void signal_error()
 * \brief Funkcja sygnalizujaca diodami odczytanie zbyt dlugiego pakietu danych
 */
void signal_error()
{
	unsigned char c=0;
	for(c=0;c<4;++c)
	{
		LED(0xf);
		delay();
		LED(0x0);
		delay();
	}
}




/*! \brief Funkcje realizujaca maszyne stanow odbiornika pakietow w formacie opisanym w instrukcji
 *  Struktura STATE jest podstawa maszyny stanow. Umozliwia realizacje przejscia ze stanu do stanu
 *  za pomoca wywolania w petli while funkcji zapisanej we wskazniku funkcyjnym fun. Zeby uruchomic
 *  maszyne nalezy umiescic funkcje state_machine w wybranym miejscu programu.
 *  Faktyczne przejscia ze stanu do stanu realizowane sa przez funkcje:
 *  - wait
 *  - read_data
 *  - read_bslash
 *  W zmiennej lokalnej ret funkcje te zapisuja nastepny stan maszyny przez przypisanie adresu jednej
 *  z wymienionych wyzej funkcji.
 */
/*!\brief Zadanie do wykonania
 *
 * Przeanalizuj podany nizej kod maszyny stanow i narysuj na jego podstawie diagram stanow.
 * Zmodyfikuj maszyne stanow w taki sposob, aby pakiety dluzsze niz 16 znakow nie byly ignorowane
 * i traktowane jako bledne. Odebranie dlugiego pakietu powinno byc sygnalizowane za pomoca
 * funkcji signal_error. Narysuj diagram zmodyfikowanej maszyny i zaimplementuj te maszyne z
 * wykorzystaniem struktury STATE. Przed testem nowej maszyny stanow nalezy zmienic zawartosc bufora buf.
 * Instrukcja w punkcie (-1-) w komentarzu do zmiennej buf. Wykorzystaj narzedzia srodowiska programistycznego
 * do sprawdzenia poprawnosci dzialania nowej maszyny. W tym celu uzyj pracy krokowej, pulapek i mozliwosci
 * podgladania stanu zmiennych.
 *
 *
 */

typedef struct _STATE
{
	struct _STATE (*fun)(void);
} STATE;


STATE IDLE();
STATE CALIBRATION();
STATE WAIT();
STATE FIRST_PULSE();
STATE NEXT_PULSE();
STATE CALIBRATION();
STATE CALCULATION();


void state_machine()
{
	STATE state={IDLE};
	while(1)
	{
		state=(state.fun());
	}
}

STATE wait()
{

}

