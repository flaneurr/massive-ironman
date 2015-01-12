package org.lejos.example;

import lejos.robotics.subsumption.*;
import lejos.nxt.*;

/*
 * Kopittelija-luokka toteuttaa kopitteluominaisuuden Lego NXT-robotille. 
 * Kopittelu on suunniteltu tehtävän kahden samanlaisen robotin välillä.
 */
public class Kopittelija {

	public boolean arbitraattoriPaalla = false;

	/*
	 * Main-metodissa alustetaan behaviorit ja käynnistetään arbitrator.
	 */
	public static void main(String[] args) throws Exception {

		// luodaan sensori-oliot
		LightSensor valo = new LightSensor(SensorPort.S2);
//		UltrasonicSensor ultra = new UltrasonicSensor(SensorPort.S1);
		TouchSensor kosketus = new TouchSensor(SensorPort.S3);
		Thread.sleep(1000);	
		
		// kalibroidaan valoisuus
		int[] kalibraatioArvot = valoKalibraatio(valo);
		
		// luodaan behavior-oliot
		Behavior b1 = new Lepo();
		Behavior b2 = new Heitto(valo, kalibraatioArvot);
//		Behavior b3 = new TestiHeitto();
//		Behavior b4 = new Etsinta(ultra);
		Behavior b5 = new KaannosVasempaan();
		Behavior b6 = new KaannosOikeaan();
		Behavior b7 = new HataSammutus(kosketus);
		Behavior[] behaviorit = { b1, b2, b5, b6, b7};
		Arbitrator arbi = new Arbitrator(behaviorit, true);
		System.out.println("Paina nappia aloittaaksesi toiminnan");
		Button.waitForPress();
		arbi.start();
		
	}
	
	/*
	 * Tämä metodi kalibroi valoisuuden siten, että pallo erotetaan yleisvalaistuksesta.
	 */
	public static int[] valoKalibraatio(LightSensor valo) {
		// luodaan lista, johon tallennetaan yleisvalaistuksen arvo, pallon valoisuuden arvo ja tieto siitä kumpi
		// näistä arvoista oli suurempi (0 jos yleisvalaistus, 1 jos pallon valoisuus )
		int[] kalibraatiot = new int[3];
		int alkuArvo = valo.readValue();
		System.out.println("Valoisuuden perusarvo on " + alkuArvo);
		kalibraatiot[0] = alkuArvo;
		System.out.println("Aseta pallo valon alle ja paina nappia, kiitos.");
		Button.waitForPress();
		int palloArvo = valo.readValue();
		System.out.println("Valoisuusarvo pallon kanssa on " + palloArvo);
		kalibraatiot[1] = palloArvo;
		if (kalibraatiot[0] > kalibraatiot[1]) {
			kalibraatiot[2] = 0;
		} else {
			kalibraatiot[2] = 1;
		}
		return kalibraatiot;
	}
}

/*
 *  HataSammutus on behavior, joka sammuttaa NXT-keskusyksikön, kun kosketussensoria painetaan.
 */
class HataSammutus implements Behavior {

	private TouchSensor kosketus;
	
	public HataSammutus(TouchSensor kosketus){
		this.kosketus = kosketus;
	}
	
	@Override
	public void action() {
		NXT.shutDown();
	}

	@Override
	public void suppress() {
	}

	@Override
	public boolean takeControl() {
		return this.kosketus.isPressed();
	}
	
}


/*
 *  Behavior, joka heittää pallon, kun pallo kulkee valosensorin ali.
 */
class Heitto implements Behavior {

	private LightSensor valo;
	private int[] kalibraatioArvot;
	private UltrasonicSensor ultra;
	private boolean suppressed;

	public Heitto(LightSensor valo, int[] kalibraatioArvot) {
		
		this.valo = valo;
		this.kalibraatioArvot = kalibraatioArvot;
		this.ultra = new UltrasonicSensor(SensorPort.S1);
	}

	@Override
	public void action() {
		suppressed = false;
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		int[] nopeusJaEtaisyys = laskeNopeus();
		Motor.A.setSpeed(nopeusJaEtaisyys[0]);
		System.out.println("Kohteen etäisyys: " + nopeusJaEtaisyys[1]);
		System.out.println("Heittovoima: " + nopeusJaEtaisyys[0]);
		if (!suppressed) {
			Motor.A.rotate(60);
			Motor.A.stop();
		}
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		if (!suppressed) {
			Motor.A.setSpeed(100);
			Motor.A.rotate(-60);		
		}
	}
	/*
	 * Tämä metodi laskee kohteen etäisyyden avulla voiman, jolla palloa on heitettävä, jotta se lentää halutun matkaa.
	 * Ensin lasketaan projektiilin nopeus lineaarisena nopeutena, jonka jälkeen se muutetaan kulmanopeudeksi.
	 */
	public int[] laskeNopeus(){
		int[] palautusArvot = new int[2];
		int ultranEtaisyys = ultra.getDistance();
		palautusArvot[1] = ultranEtaisyys;
		int tahtaysEtaisyys = ultranEtaisyys + 15; // saatu etäisyys lisättynä etäisyys robotin päästä kiinniottokouraan
		double kulma = 45; // heittokulma
		double korkeus = 30; // heittokorkeus
		double putoamiskiihtyvyys = 981;
		int sade = 19; // kouran säde
		int lineaarinenNopeus = (int) Math.ceil((Math.abs((tahtaysEtaisyys * Math.sqrt(putoamiskiihtyvyys)*Math.sqrt(1/Math.cos(kulma)))/Math.sqrt(2*tahtaysEtaisyys*Math.sin(kulma)+2*korkeus*Math.cos(kulma)))));
		int kulmaNopeus = lineaarinenNopeus / sade;
		palautusArvot[0] = (int)Math.ceil(kulmaNopeus * (180/Math.PI) * 0.6); // muutetaan radiaanit asteiksi ja otetaan huomioon moottorin rattaat
		return palautusArvot;
	}

	@Override
	public void suppress() {
		suppressed = true;

	}
	/*
	 * Palauttaa true, jos saatu valoisuusarvo tarpeeksi lähellä pallon valoisuusarvoa.
	 */
	@Override
	public boolean takeControl() {
		int valoArvo = valo.readValue();
		if (kalibraatioArvot[2] == 0) {
			return valoArvo <= kalibraatioArvot[0] - ((kalibraatioArvot[0] - kalibraatioArvot[1])/2);
		} 
		return valoArvo > kalibraatioArvot[0] + ((kalibraatioArvot[1] - kalibraatioArvot[0])/2);
	}
}


/*
 * Lepo-behaviorin ollessa aktiivisena robotti ei tee mitään ja odottaa toimintasignaalia toiselta behaviorilta.
 */
class Lepo implements Behavior {

	@Override
	public void action() {
	}

	@Override
	public void suppress() {
	}

	@Override
	public boolean takeControl() {
		return true;
	}

}


/*
 * Tämä behavior saa aikaan sen, että painettaessa vasenta nuolinäppäintä robotti kääntyy vasemmalle akselinsa ympäri.
 */
class KaannosVasempaan implements Behavior {

	private boolean suppressed;
	
	public KaannosVasempaan (){
		this.suppressed  = true;
	}
	
	@Override
	public void action() {
		this.suppressed = false;
		Motor.B.setSpeed(150);
		while (Button.LEFT.isPressed() && !suppressed){
			Motor.B.backward();
		}
		Motor.B.stop();
	}

	@Override
	public void suppress() {
		this.suppressed = true;
		
	}

	@Override
	public boolean takeControl() {
		return Button.LEFT.isPressed();
	}
	
}

/*
 * Tämä behavior saa aikaan sen, että painettaessa oikeaa nuolinäppäintä robotti kääntyy oikealle akselinsa ympäri.
 */
class KaannosOikeaan implements Behavior {
	
	private boolean suppressed;
	
	public KaannosOikeaan (){
		this.suppressed  = true;
	}
	@Override
	public void action() {
		this.suppressed = false;
		Motor.B.setSpeed(150);
		while (Button.RIGHT.isPressed() && !suppressed){
			Motor.B.forward();
		}
		Motor.B.stop();
		
	}

	@Override
	public void suppress() {
		this.suppressed = true;
	}

	@Override
	public boolean takeControl() {
		return Button.RIGHT.isPressed();
	}
	
}

/*
 * Tämä luokka toteuttaa kohteen etsinnän. Robotti kääntyy akselinsa ympäri kunnes löytää tarpeeksi lähellä olevan kohteen.
 */
//class Etsinta implements Behavior {
//
//	private boolean suppressed;
//	private UltrasonicSensor ultra;
//	
//	public Etsinta (UltrasonicSensor ultra){
//		this.suppressed = true;
//		this.ultra = ultra;
//	}
//	
//	@Override
//	public void action() {
//		this.suppressed = false;
//		Motor.B.setSpeed(100);
//		Motor.B.forward();
//		System.out.println("Etsitaan kohdetta...");
//		while(ultra.getDistance() > 50 && !suppressed){
//			System.out.println(ultra.getDistance());
//		}
//		Motor.B.stop();
//		LCD.clear();
//		}
//
//	@Override
//	public void suppress() {
//		Motor.B.stop();
//		this.suppressed = true;		
//	}
//
//	@Override
//	public boolean takeControl() {
//		return this.ultra.getDistance() > 50; // robon maksimiheittoetäisyys
//	}
//	
//}


/*
 *  Luokka pallon heittämisen testaamiseksi. Yksinkertainen behvior, joka kääntää moottoria pallon heittämiseksi ja kääntää moottorin takaisin.
 */
//class TestiHeitto implements Behavior {
//
//	private boolean suppressed;
//
//	public TestiHeitto(){
//		this.suppressed = true;
//	}
//	
//	@Override
//	public void action() {
//		this.suppressed = false;
//		while (!this.suppressed){
//		Motor.A.setSpeed(900);
//		Motor.A.rotate(60);
//		Motor.A.stop();
//		try {
//			Thread.sleep(1000);
//		} catch (InterruptedException e) {
//			e.printStackTrace();
//		}
//		Motor.A.setSpeed(50);
//		Motor.A.rotate(-60);
//		Button.waitForPress();
//		}
//	}
//
//	@Override
//	public void suppress() {
//		this.suppressed = true;
//
//	}
//
//	@Override
//	public boolean takeControl() {
//		return true;
//	}
//}



