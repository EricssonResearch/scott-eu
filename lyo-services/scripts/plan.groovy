@Grab('io.github.http-builder-ng:http-builder-ng-core:1.0.3')
@Grab(group = 'org.eclipse.lyo.oslc4j.core', module = 'oslc4j-core', version = '2.4.0')
@Grab(group = 'org.eclipse.lyo.oslc4j.core', module = 'oslc4j-jena-provider', version = '2.4.0')
@Grab(group = 'org.slf4j', module = 'slf4j-simple', version = '1.7.25')

import groovyx.net.http.ChainedHttpConfig
import groovyx.net.http.HttpBuilder
import groovyx.net.http.ToServer
import org.apache.jena.rdf.model.Model
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper

println("Running planning script...")

String endpoint = 'http://localhost:8080/'
if (args.length > 0 && args[0] == "debug") {
    endpoint = 'http://localhost:8180/'
}

HttpBuilder client = HttpBuilder.configure {
    request.uri = endpoint
    request.encoder('text/turtle') { ChainedHttpConfig config, ToServer req ->
        // TODO optimise directly to bytes
        // TODO deduplicate
        def rdf = serialiseModel(JenaModelHelper.createJenaModel([config.request.body] as Object[]), RDFFormat.TURTLE_PRETTY)
        req.toServer(new ByteArrayInputStream(rdf.bytes))
    }
}

triggerPlanning(client)


// HELPER METHODS

private static triggerPlanning(client) {
    def result = client.post {
        request.uri.path = '/services/admin/plan_trigger'
        request.contentType = 'text/turtle'
        request.body = null
        response.success {
            println("Triggered default planning mode")
        }
    }
    result
}

private static String serialiseModel(Model m, RDFFormat f) {
    StringWriter writer = new StringWriter()
    RDFDataMgr.write(writer, m, f)
    return writer.toString()
}
