package eu.scott.warehouse.lib

import org.eclipse.lyo.oslc4j.core.model.IExtendedResource

interface ResourceArray {
    fun asResourceArray(): Array<Object>
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
