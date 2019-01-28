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
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper

HttpBuilder client = HttpBuilder.configure {
    request.uri = 'http://localhost:8081/'
    request.encoder('text/turtle') { ChainedHttpConfig config, ToServer req ->
        // TODO optimise directly to bytes
        def rdf = serialiseModel(JenaModelHelper.createJenaModel([config.request.body] as Object[]), RDFFormat.TURTLE_PRETTY)
        req.toServer(new ByteArrayInputStream(rdf.bytes))
    }
}

enum Operation {
    SINGLE,
    MULTI
}

Operation operation = null

if (args[0] == null) {
    throw new IllegalArgumentException("Operation must be selected: SINGLE or MULTI")
} else {
    if (args[0].toUpperCase() == "SINGLE") {
        operation = Operation.SINGLE
    } else if (args[0].toUpperCase() == "MULTI") {
        operation = Operation.MULTI
    } else {
        throw new IllegalArgumentException("Operation must be selected: SINGLE or MULTI")
    }
}

if (operation == Operation.SINGLE) {
    String id = args[1] ?: 'rb-1'
    ServiceProvider serviceProvider = createServiceProvider(id)

    createTwin(client, serviceProvider)
} else if (operation == Operation.MULTI) {
    int from = Integer.parseInt(args[1])
    int to = Integer.parseInt(args[2])

    for (id in from..to) {
        ServiceProvider sp = createServiceProvider("rb-$id")
        createTwin(client, sp)
    }
}



// HELPER METHODS

private static ServiceProvider createServiceProvider(String id) {
    ServiceProvider serviceProvider = new ServiceProvider()
    serviceProvider.identifier = id
    serviceProvider.created = new Date()
    serviceProvider.description = "Remotely created SP '$id'"
    serviceProvider.title = "${id.toUpperCase()}"
    serviceProvider
}

private void createTwin(HttpBuilder client, ServiceProvider serviceProvider) {
    def result = client.post {
        request.uri.path = '/services/admin/initRDF'
        request.contentType = 'text/turtle'
        request.body = serviceProvider
        response.success {
            println("Created a twin '$serviceProvider.identifier'")
        }
    }
}

private static String serialiseModel(Model m, RDFFormat f) {
    StringWriter writer = new StringWriter()
    RDFDataMgr.write(writer, m, f)
    return writer.toString()
}
