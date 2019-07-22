package se.ericsson.cf.scott.sandbox.svc.location.xtra

import org.apache.jena.rdf.model.Model
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.slf4j.Logger
import org.slf4j.LoggerFactory

object LyoModels {
    private val log: Logger = LoggerFactory.getLogger(LyoModels::class.java)
    fun <T : IExtendedResource> hasResource(model: Model?, clazz: Class<T>): Boolean = try {
        val resources = JenaModelHelper.unmarshal(model, clazz)
        resources?.isNotEmpty() ?: false
    } catch (e: Exception) {
        log.warn("Error processing Jena model while looking for an instance of ${clazz.simpleName}", e)
        false
    }
}
