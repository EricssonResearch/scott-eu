/**
 * https://github.com/soartech/jsoar/wiki/JSoarUsersGuide
 */
//package myJsoarAgent;
import java.util.*;
import java.util.Iterator;
import java.io.OutputStreamWriter;
import org.jsoar.runtime.ThreadedAgent;
import org.jsoar.util.commands.SoarCommands;
import org.jsoar.kernel.*;
import org.jsoar.kernel.memory.Wme;
import org.jsoar.kernel.RunType;
import java.util.Set;
import org.jsoar.kernel.io.InputOutput;
import org.jsoar.kernel.io.InputWme;
import org.jsoar.kernel.memory.WmeImpl;
import py4j.GatewayServer; //http://stackoverflow.com/questions/22386399/simplest-example-with-py4j
import org.jsoar.kernel.symbols.SymbolFactory;
import org.jsoar.kernel.symbols.StringSymbolImpl;
import org.jsoar.kernel.symbols.DoubleSymbolImpl;
import org.jsoar.kernel.symbols.Symbol;
import org.jsoar.kernel.symbols.SymbolFactoryImpl;
import org.jsoar.kernel.symbols.Identifier;

/**
General SOAR agent in java prepared to be accessed using p4j
	
 *	 @author Klaus Raizer
 *
 */
public class P4jSoarAgent {

public Agent agent=null;
public String agentName="";
public String stringWMEs="";
public String WMEsUnder="";
public Hashtable<String,InputWme> InputWmeHash = new Hashtable<String,InputWme>();

 	public int newSoarAgent(String name) {
		agentName=name;
		agent = new Agent();
		agent.setName(agentName);
		agent.getPrinter().pushWriter(new OutputStreamWriter(System.out));
		System.out.println("Created a soar agent named: "+agentName);
    		return 1;
  	}

 	public int loadRules(String rulesPath) {
		if(agent==null){
			System.out.println("Error: agent==null");
			return 0;
		}
		else{
			try {			
				SoarCommands.source(agent.getInterpreter(), rulesPath);
			} catch (org.jsoar.kernel.SoarException se) {
			      System.out.println ("Exception " + se);
			}
			System.out.println("Loaded rules to agent '"+agentName+"' from source: "+rulesPath);
	    		return 1;
		}
  	}

	public void initialize(){agent.initialize();}
	public void runForever(){agent.runForever();}
	public void dispose(){agent.dispose();}
	public String getAllWmesInRete(){return agent.getAllWmesInRete().toString();}
	
	public void runFor(long n){
		agent.runFor(n,RunType.DECISIONS);
	}

	public String getInputLink(){
		String inputLinkIdentifierString="";
		SymbolFactory sf = agent.getSymbols();
		SymbolFactoryImpl sfi = (SymbolFactoryImpl)sf;
		Identifier inputLink = agent.getInputOutput().getInputLink();
		inputLinkIdentifierString=inputLink.getNameLetter()+String.valueOf(inputLink.getNameNumber());
		return inputLinkIdentifierString;
	}

	

	/**
	* Creates the passed graph structure in the agent's Input link.
	* If the root node already exists, it overwrites it.
	*/
	public void addSensorReadingsToInput(String string_content){
	
		SymbolFactory sf = agent.getSymbols();
		SymbolFactoryImpl sfi = (SymbolFactoryImpl)sf;
		Identifier inputLink = agent.getInputOutput().getInputLink();

		String[] triples = string_content.split(";");
		//System.out.println("--- string_content ---");
		//System.out.println("triples.length: "+triples.length);
		//int index = 0;
		for(int index=0; index<triples.length; index++){
	//		System.out.println(triples[index]);
			String[] triple = triples[index].split(",");

	//		System.out.println(triple[0]);
	//		System.out.println(triple[1]);
	//		System.out.println(triple[2]);

	//		System.out.println("--------------");
			//TODO: Should check if Node before doing this?
			String identifierString=triple[0];
			char name_letter =identifierString.charAt(0);
			long name_number=Character.getNumericValue(identifierString.charAt(1));

			Identifier identifier  = sfi.findOrCreateIdentifierExact(name_letter,name_number);

			Symbol attribute=sf.createString(triple[1]);

			//Should check if Node before doing this. If not, create as Symbol
			String valueString=triple[2];

			Boolean valueIsNode=isSymbolNode(valueString);
	
			if(valueIsNode){
				name_letter =valueString.charAt(0);
				name_number =Character.getNumericValue(valueString.charAt(1));
				Identifier value = sfi.findOrCreateIdentifierExact(name_letter,name_number);

				agent.getInputOutput().addInputWme(identifier, attribute, value);
			}else{
				Symbol value = sf.createString(valueString);
				agent.getInputOutput().addInputWme(identifier, attribute, value);
			}

		}						

			//(identifier ^attribute value)

			//(I2,blah,T1)
	       	
		//addInputWme(Identifier id, Symbol attr, Symbol value)	

	
	}

	public String UpdateWme(String wmeKey,String value){//TODO values could be of other types, such as floats

	//recovers WME from hash table using wmeKey as key
		InputWme wme=InputWmeHash.get(wmeKey);
	//update the WME with new given value
	//TODO: create identifier if given something like A1, C3 etc
		SymbolFactory sf = agent.getSymbols();
		SymbolFactoryImpl sfi = (SymbolFactoryImpl)sf;
		Symbol val=sfi.createString(value);
		wme.update(val);
	//build a new key
		Identifier newIdent=wme.getIdentifier();
		StringSymbolImpl newStringAttrib=(StringSymbolImpl)wme.getAttribute();
		StringSymbolImpl newStringValue=(StringSymbolImpl)wme.getValue();
		char name_letter=newIdent.getNameLetter();
		long name_number=newIdent.getNameNumber();
		String newWmeKey=""+name_letter+""+name_number+","+newStringAttrib.getValue()+","+newStringValue.getValue()+"";
		//System.out.println("*******"+newWmeKey+"*************************************");	
	//update key on hashtable
		InputWmeHash.remove(wmeKey);
		InputWmeHash.put(newWmeKey,wme);
	//return new wmeKey

		return newWmeKey;
	}


	public String UpdateWme(String wmeKey,double value){//TODO values could be of other types, such as double

	//recovers WME from hash table using wmeKey as key
		InputWme wme=InputWmeHash.get(wmeKey);
	//update the WME with new given value
	//TODO: create identifier if given something like A1, C3 etc
		SymbolFactory sf = agent.getSymbols();
		SymbolFactoryImpl sfi = (SymbolFactoryImpl)sf;
		Symbol val=sfi.createDouble(value);
		wme.update(val);
	//build a new key
		Identifier newIdent=wme.getIdentifier();
		StringSymbolImpl newStringAttrib=(StringSymbolImpl)wme.getAttribute();

		DoubleSymbolImpl newDoubleValue=(DoubleSymbolImpl)wme.getValue(); //TODO should be double
		char name_letter=newIdent.getNameLetter();
		long name_number=newIdent.getNameNumber();
		String newWmeKey=""+name_letter+""+name_number+","+newStringAttrib.getValue()+","+newDoubleValue.toString()+"";
		//System.out.println("*******"+newWmeKey+"*************************************");	
	//update key on hashtable
		InputWmeHash.remove(wmeKey);
		InputWmeHash.put(newWmeKey,wme);
	//return new wmeKey

		return newWmeKey;
	}


	public String InputWme(String identifier, String attribute, String value){
		//System.out.println("********************************************");	
		//System.out.println(identifier+","+attribute+","+value);

		
		SymbolFactory sf = agent.getSymbols();
		SymbolFactoryImpl sfi = (SymbolFactoryImpl)sf;
		Identifier inputLink = agent.getInputOutput().getInputLink();
		

		//Break identifier down 
		char name_letter = identifier.charAt(0);
		long name_number = Long.parseLong(identifier.substring(1,identifier.length()), 10);

		Identifier id = sfi.findOrCreateIdentifierExact(name_letter,name_number);
		Symbol att=sfi.createString(attribute);
		
		//TODO: create identifier if given something like A1, C3 etc
//-----------------------------------
		InputWme newInputWme=null;
		Boolean valueIsNode=isSymbolNode(value);		
		if(valueIsNode){
			name_letter =value.charAt(0);
			name_number =Character.getNumericValue(value.charAt(1));
			Identifier val = sfi.findOrCreateIdentifierExact(name_letter,name_number);

			//agent.getInputOutput().addInputWme(identifier, attribute, valueIdentifier);
			newInputWme=agent.getInputOutput().addInputWme(id, att, val);
		}else{
			Symbol val = sfi.createString(value);
			//agent.getInputOutput().addInputWme(identifier, attribute, valueSymbol);
			newInputWme=agent.getInputOutput().addInputWme(id, att, val);
		}

//------------------------------		


		//Symbol val=sfi.createString(value);

		//InputWme newInputWme=agent.getInputOutput().addInputWme(id, att, val);

		Identifier newIdent=newInputWme.getIdentifier();
		StringSymbolImpl newStringAttrib=(StringSymbolImpl)newInputWme.getAttribute();
		StringSymbolImpl newStringValue=null;
		String valueAsString="";
		if(valueIsNode){
			Identifier valIdent=newInputWme.getIdentifier();
			name_letter=newIdent.getNameLetter();
			name_number=newIdent.getNameNumber();
			valueAsString=""+name_letter+""+name_number+"";
		}else{
			newStringValue=(StringSymbolImpl)newInputWme.getValue();
			valueAsString=newStringValue.getValue();
		}		



		name_letter=newIdent.getNameLetter();
		name_number=newIdent.getNameNumber();
		String hashKey=""+name_letter+""+name_number+","+newStringAttrib.getValue()+","+valueAsString+"";

		InputWmeHash.put(hashKey,newInputWme);

		return hashKey;
	}



	public String getWMEsUnder(char name_letter,long name_number){
		SymbolFactory sf = agent.getSymbols();
		//name_letter='S';
		//name_number=1;
		int level=0;
		Identifier id=sf.findIdentifier(name_letter, name_number);
		WMEsUnder="";
		getWMEs(id,level);
		
		//System.out.println("==name_letter:"+name_letter+"======name_number:"+name_number+"===========");
		return WMEsUnder;

	}


	public void getWMEs(Identifier id, int level) {
		Iterator<Wme> It = id.getWmes();
		while (It.hasNext()) {
			Wme wme = It.next();
			Identifier idd = wme.getIdentifier();
			Symbol a = wme.getAttribute();
			Symbol v = wme.getValue();
			Identifier testv = v.asIdentifier();
			for (int i=0;i<level;i++) System.out.print("   ");
				if(WMEsUnder.equals("")){
					WMEsUnder=idd.toString()+","+a.toString()+","+v.toString();
				}else{
					WMEsUnder=WMEsUnder+";"+idd.toString()+","+a.toString()+","+v.toString();
				}
				if (testv != null) {getWMEs(testv,level+1);}
		}
	}







	public void printAllWME() {
		SymbolFactory sf = agent.getSymbols();
		char name_letter='S';
		long name_number=1;
		int level=0;
		Identifier id=sf.findIdentifier(name_letter, name_number);
		System.out.println("--------------------------------");
		stringWMEs="";
		printWME(id,level);
		System.out.println("-----------------"+stringWMEs+"---------------");	
			
	}

	

 	public void printWME(Identifier id, int level) {
		Iterator<Wme> It = id.getWmes();
		while (It.hasNext()) {
		    Wme wme = It.next();
		    Identifier idd = wme.getIdentifier();
		    Symbol a = wme.getAttribute();
		    Symbol v = wme.getValue();
		    Identifier testv = v.asIdentifier();
		    for (int i=0;i<level;i++) System.out.print("   ");
		    if (testv != null) {
		        System.out.print("("+idd.toString()+","+a.toString()+","+v.toString()+")\n");
			stringWMEs=stringWMEs+";("+idd.toString()+","+a.toString()+","+v.toString()+")";
		        printWME(testv,level+1);
		    }
		    else {
			System.out.print("("+idd.toString()+","+a.toString()+","+v.toString()+")\n");
			stringWMEs=stringWMEs+";("+idd.toString()+","+a.toString()+","+v.toString()+")";}
		}
	    }

/**
 	public void printWME(Identifier id, int level) {
		Iterator<Wme> It = id.getWmes();
		while (It.hasNext()) {
		    Wme wme = It.next();
		    Identifier idd = wme.getIdentifier();
		    Symbol a = wme.getAttribute();
		    Symbol v = wme.getValue();
		    Identifier testv = v.asIdentifier();
		    for (int i=0;i<level;i++) System.out.print("   ");
		    if (testv != null) {
		        System.out.print("("+idd.toString()+","+a.toString()+","+v.toString()+")\n");
		        printWME(testv,level+1);
		    }
		    else System.out.print("("+idd.toString()+","+a.toString()+","+v.toString()+")\n");
		}
	    }
*/




/**
    public Set<Wme> getAllWmesInRete()
    {
        return new LinkedHashSet<Wme>(rete.getAllWmes());
    }
    //
    // Returns this agent's input/output interface
    // 
    // @return the agent's input/output interface
    //
    public InputOutput getInputOutput()
    {
        return io;
    }

    public Identifier getOutputLink() {
        Identifier ol = agent.getInputOutput().getOutputLink();
        return(ol);
    }
    
    public Identifier getInputLink() {
        Identifier il = agent.getInputOutput().getInputLink();
        return(il);
    }


*/
	public static boolean isInteger(String s) {
	    return isInteger(s,10);
	}

	public static boolean isInteger(String s, int radix) {
	    if(s.isEmpty()) return false;
	    for(int i = 0; i < s.length(); i++) {
		if(i == 0 && s.charAt(i) == '-') {
		    if(s.length() == 1) return false;
		    else continue;
		}
		if(Character.digit(s.charAt(i),radix) < 0) return false;
	    }
	    return true;
	}


	public static boolean isSymbolNode(String valueString){
	Boolean valueIsNode=true;
	Integer part = 1;

	Character ch = valueString.charAt(0);
	if(!Character.isUpperCase(ch)){
		valueIsNode=false;
	}else{
		for(int i=1; i<valueString.length(); i++){
			ch = valueString.charAt(i);
			//System.out.println("valueString["+i+"]: "+ch);
			Boolean upperCaseCh=Character.isUpperCase(ch);
			Boolean integerCh=isInteger(Character.toString(ch));
			//System.out.println("upperCaseCh: "+upperCaseCh+" integerCh: "+integerCh);
			if(part==1){
				if(!upperCaseCh && integerCh){
					part=2;
				}else if(!upperCaseCh && !integerCh){
					valueIsNode=false;
					break;
				}
			}else{
				if(!integerCh){
					valueIsNode=false;
					break;					
				}
			}
		}
	}

	//System.out.println("valueIsNode: "+valueIsNode);
	return valueIsNode;
	}








	/**
	 * @param args
	 */
	public static void main(String[] args) {


		P4jSoarAgent app = new P4jSoarAgent();
    		// app is now the gateway.entry_point
    		GatewayServer server = new GatewayServer(app);//,25333);
		System.out.println("Soar agent p4j gateway server is running in java...");
    		server.start();


/**

		Agent agent = new Agent();
		agent.setName("My SOAR Agent");
		agent.getPrinter().pushWriter(new OutputStreamWriter(System.out));
		System.out.println("Soar rules file: "+args[0]);
		
		try {			
			SoarCommands.source(agent.getInterpreter(), args[0]);
		} catch (org.jsoar.kernel.SoarException se) {
		      System.out.println ("Exception " + se);
		}

		agent.initialize();



		agent.runForever(); // Call blocks until agent interrupts or halts
		
		

		agent.dispose();*/

				
	}


}


















