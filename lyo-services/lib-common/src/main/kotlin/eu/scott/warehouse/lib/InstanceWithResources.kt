package eu.scott.warehouse.lib

import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.RDFDataMgr
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper

interface ModelProvider {
    fun asModel(): Model
}

interface ResourceArray : ModelProvider {
    fun asResourceArray(): Array<Object>

    override fun asModel(): Model {
        return JenaModelHelper.createJenaModel(asResourceArray())
    }
}

data class InstanceWithResources<T : IExtendedResource>(val instance: T,
                                                        val resources: Collection<IExtendedResource>) :
    ResourceArray {
    override fun asResourceArray(): Array<Object> {
        val resourceArray: MutableList<Object> = ArrayList()
        resourceArray.add(instance as Object)
        resourceArray.addAll(resources as Collection<Object>)
        return resourceArray.toTypedArray()
    }
}

data class InstanceWithModel<T : IExtendedResource>(val instance: T,
                                                        val model: Model) :
    ModelProvider {
    override fun asModel(): Model {
        val instanceModel: Model = JenaModelHelper.createJenaModel(arrayOf(instance as Object))
        instanceModel.add(model)
        return instanceModel
    }
}

data class InstanceMultiWithResources<T : IExtendedResource>(val instance: Collection<T>,
                                                             val resources: Collection<IExtendedResource>) :
    ResourceArray {
    override fun asResourceArray(): Array<Object> {
        val resourceArray: MutableList<Object> = ArrayList()
        resourceArray.addAll(instance as Collection<Object>)
        resourceArray.addAll(resources as Collection<Object>)
        return resourceArray.toTypedArray()
    }
}
